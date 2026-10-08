/* Small force actions routed through the native React controls, including their limits. */
(() => {
  'use strict';
  const frame = () => new Promise(resolve => requestAnimationFrame(resolve));
  const widgets = new WeakMap();
  function controls(panel) {
    const sections = [...panel.querySelectorAll('.brace-force-section')];
    const load = sections.find(s => s.firstElementChild.textContent.trim() === 'Hand Force');
    const exert = sections.find(s => s.firstElementChild.textContent.trim() === 'Exert');
    return { load, exert, mode: exert?.querySelector('input[type=checkbox]') };
  }
  function slider(section, hand, axis) {
    return [...(section?.querySelectorAll('[role=slider]') ?? [])].find(s => {
      const text = s.getAttribute('aria-label')?.toLowerCase() ?? '';
      return text.includes(`${hand} hand`) && text.endsWith(`${axis.toLowerCase()} (n)`);
    });
  }
  async function write(panel, element, value) {
    if (!element) throw new Error('This force control is unavailable.');
    if (Math.abs(Number(element.getAttribute('aria-valuenow')) - value) < .001) return;
    if (element.hasAttribute('data-disabled')) throw new Error('Wait for the simulation to load.');
    const min = Number(element.getAttribute('aria-valuemin'));
    const max = Number(element.getAttribute('aria-valuemax'));
    if (value < min || value > max) throw new Error(`Use a force between ${min} and ${max} N.`);
    const section = element.closest('.brace-force-section');
    // A collapsed native section must open before its slider has measurable geometry.
    if (section.children[1]?.getAttribute('aria-hidden') === 'true') {
      section.firstElementChild.click();
      await new Promise(resolve => setTimeout(resolve, 230));
    }
    const track = element.closest('.mantine-Slider-root').querySelector('.mantine-Slider-trackContainer');
    const box = track.getBoundingClientRect();
    if (!box.width || !box.height) throw new Error('Force controls are not ready.');
    const x = box.left + (value - min) / (max - min) * box.width;
    track.dispatchEvent(new MouseEvent('mousedown', { bubbles: true, clientX: x, clientY: box.top + box.height / 2, button: 0 }));
    await frame();
    document.dispatchEvent(new MouseEvent('mouseup', { bubbles: true }));
    // React may publish the command snapshot after another simulation frame.
    for (let attempt = 0; attempt < 60; attempt++) {
      await frame();
      const actual=Number(element.getAttribute('aria-valuenow'));
      if (Math.abs(actual - value) < .001) break;
      // Correct track/thumbnail pixel rounding through the native accessible step handler.
      element.dispatchEvent(new KeyboardEvent('keydown',{key:actual<value ? 'ArrowRight' : 'ArrowLeft',bubbles:true}));
    }
    if (Math.abs(Number(element.getAttribute('aria-valuenow')) - value) > .001) throw new Error('The force was not applied. Please try again.');
  }
  async function zero(panel, section) {
    for (const element of section?.querySelectorAll('[role=slider]') ?? []) {
      if (Number(element.getAttribute('aria-valuenow')) !== 0) await write(panel, element, 0);
    }
  }
  function mount(panel) {
    if (widgets.has(panel) || !controls(panel).load) return;
    const disclosure = panel.querySelector('.brace-disclosure');
    if (!disclosure) return;
    const widget = document.createElement('section');
    widget.className = 'brace-force-actions';
    widget.setAttribute('aria-label', 'Quick force controls');
    widget.innerHTML = `<fieldset><legend class="brace-sr-only">Force action</legend><div class="brace-force-modes" role="group" aria-label="Force mode"><button type="button" data-mode="compensate" aria-pressed="true"><strong>Compensate</strong><span>Resist a hand load</span></button><button type="button" data-mode="exert" aria-pressed="false"><strong>Exert</strong><span>Ask the robot to push</span></button></div><div class="brace-force-target"><label>Hand<select aria-label="Force hand"><option value="right">Right hand</option><option value="left">Left hand</option><option value="both">Both hands</option></select></label><label>Direction<select aria-label="Force direction"><option value="X:1">+X · Forward</option><option value="X:-1">−X · Backward</option><option value="Y:1">+Y · Left</option><option value="Y:-1">−Y · Right</option><option value="Z:1">+Z · Up</option><option value="Z:-1">−Z · Down</option></select></label></div><p class="brace-force-frame"></p><section class="brace-gauge-card" aria-label="Live force comparison" hidden><div class="brace-gauge-heading"><strong>Force comparison</strong><span>LIVE</span></div><p>Blue shows the effective target; amber shows the exerted force.</p><div class="brace-gauge-hands"></div><small class="brace-gauge-note">Choose a force to begin. Readings come from the virtual contact, in newtons.</small></section><div class="brace-force-presets" role="group" aria-label="Preset force values"></div><form class="brace-force-custom"><label>Custom force (N)<input type="number" aria-label="Custom force in newtons" value="5" min="0" step="0.5" required></label><button type="submit">Apply force</button></form><div class="brace-force-footer"><button type="button" class="brace-force-clear">Clear force</button><button type="button" class="brace-force-detail-toggle" aria-expanded="false">Individual axes</button></div><p class="brace-force-status" role="status" aria-live="polite">No force applied.</p></fieldset>`;
    disclosure.after(widget);
    const state = { mode: 'compensate', busy: false, activeValue: 0 };
    const fieldset = widget.querySelector('fieldset');
    const hand = widget.querySelector('[aria-label="Force hand"]');
    const direction = widget.querySelector('[aria-label="Force direction"]');
    const custom = widget.querySelector('input[type=number]');
    const status = widget.querySelector('[role=status]');
    const presetGroup = widget.querySelector('.brace-force-presets');
    let lastGaugeUpdate = 0;
    window.addEventListener('brace:force-reading', event => {
      if (window.BraceExert) window.BraceExert.reading = event.detail;
      const now = performance.now(); if (now-lastGaugeUpdate < 100) return; lastGaugeUpdate = now;
      const {commanded,measured} = event.detail;
      const raw = window.BraceGym?.force?.()?.raw ?? [];
      // The exert command is [task mode, left XYZ, right XYZ]. The mode is not a force.
      const requested = raw.length === 7 ? raw.slice(1) : raw;
      const target = widget.querySelector('.brace-gauge-hands');
      target.replaceChildren();
      for (let h=0;h<2;h++) {
        const cmd=Math.hypot(...commanded.slice(h*3,h*3+3)), actual=measured[h] ?? 0;
        if (cmd < .01 && Math.hypot(...requested.slice(h*3,h*3+3)) < .01) continue;
        const req=Math.hypot(...requested.slice(h*3,h*3+3));
        const row=document.createElement('div'); row.className='brace-gauge-hand';
        row.innerHTML=`<strong>${h===0 ? 'Left' : 'Right'} hand</strong><small class="brace-requested">Requested ${req.toFixed(1)} N</small><div class="brace-gauge-values"><span>Effective <b>${cmd.toFixed(1)} N</b></span><span>Measured <b>${actual.toFixed(1)} N</b></span></div><div class="brace-gauge-track"><i style="width:${Math.min(100,Math.max(0,actual)/9*100)}%"></i><em style="left:${Math.min(100,cmd/9*100)}%"></em></div>`;
        target.append(row);
      }
      widget.querySelector('.brace-gauge-note').textContent=target.children.length ? 'The effective target adapts automatically to movement and balance, then applies the checkpoint’s reach and effort limits. Your requested force stays unchanged. Measured force comes from the virtual contact; the hands stay near their IK pose.' : 'Choose a force to begin. Readings come from the virtual contact, in newtons.';
    });
    function maximum() {
      const section = controls(panel)[state.mode === 'exert' ? 'exert' : 'load'];
      const limit = Number(slider(section, 'right', 'X')?.getAttribute('aria-valuemax') ?? 0);
      return state.mode === 'compensate' && hand.value === 'both' ? Math.min(limit, 20) : limit;
    }
    function repaint() {
      fieldset.disabled=state.busy || !window.BraceGym?.ready;
      const max = maximum();
      custom.max = String(max);
      custom.step = state.mode === 'exert' ? '0.25' : '0.5';
      custom.value = String(Math.min(Number(custom.value) || 0, max));
      widget.querySelectorAll('[data-mode]').forEach(b => {
        b.setAttribute('aria-pressed', String(b.dataset.mode === state.mode));
        b.disabled = b.dataset.mode === 'exert' && !controls(panel).exert;
      });
      const world = state.mode === 'compensate';
      widget.querySelector('.brace-gauge-card').hidden = world;
      if (window.BraceExert) window.BraceExert.enabled = !world && window.BraceGym?.selected === 'none';
      widget.querySelector('.brace-force-frame').textContent = world ? 'Directions use the world axes. Values are per hand.' : 'Local torso heading (yaw only). +Z is up; values are per hand.';
      direction.options[0].text = world ? '+X · World X' : '+X · Forward';
      direction.options[1].text = world ? '−X · World X' : '−X · Backward';
      direction.options[2].text = world ? '+Y · World Y' : '+Y · Left';
      direction.options[3].text = world ? '−Y · World Y' : '−Y · Right';
      presetGroup.replaceChildren();
      for (const value of state.mode === 'exert' ? [2, 5, 8] : [5, 10, 20]) {
        if (value > max) continue;
        const button = document.createElement('button');
        button.type = 'button';
        button.textContent = `${value} N`;
        button.setAttribute('aria-label', `Apply ${value} newtons`);
        button.setAttribute('aria-pressed', String(state.activeValue === value));
        button.addEventListener('click', () => { custom.value = String(value); apply(value); });
        presetGroup.append(button);
      }
    }
    async function run(action) {
      if (state.busy) return;
      state.busy = true;
      fieldset.disabled = true;
      panel.classList.add('brace-writing-forces');
      const previousFocus = document.activeElement;
      try { await action(); }
      catch (error) { status.textContent = error.message; status.dataset.error = 'true'; }
      finally {
        panel.classList.remove('brace-writing-forces');
        fieldset.disabled = false;
        state.busy = false;
        repaint();
        if (previousFocus?.isConnected) previousFocus.focus({ preventScroll: true });
      }
    }
    async function clear() {
      const native = controls(panel);
      await zero(panel, native.load);
      if (native.mode?.checked) {
        await zero(panel, native.exert);
        native.mode.click();
        await frame();
      }
      status.textContent = 'No force applied.';
      state.activeValue = 0;
      delete status.dataset.error;
      disclosure.textContent = 'Advance Force Controls';
      presetGroup.querySelectorAll('button').forEach(b => b.setAttribute('aria-pressed', 'false'));
    }
    window.addEventListener('brace:ready',repaint);
    window.addEventListener('brace:gym-reset', () => { state.mode='compensate'; state.activeValue=0; disclosure.textContent='Advance Force Controls'; status.textContent='No force applied.'; delete status.dataset.error; repaint(); });
    window.braceClearForces = async () => {
      while (state.busy) await frame();
      return run(clear);
    };
    function apply(value) {
      if (!Number.isFinite(value) || value < 0 || value > maximum() || Math.abs(value / Number(custom.step) - Math.round(value / Number(custom.step))) > .001) {
        custom.reportValidity();
        status.textContent = `Enter 0–${maximum()} N in ${custom.step} N steps.`;
        status.dataset.error = 'true';
        return;
      }
      window.BraceGym?.remove();
      run(async () => {
        await clear();
        if (!value) return;
        const native = controls(panel);
        if (state.mode === 'exert') {
          if (native.mode.disabled) throw new Error('Wait for the simulation to load.');
          native.mode.click();
          await frame();
          await frame();
          await zero(panel, native.exert);
        }
        const section = state.mode === 'exert' ? native.exert : native.load;
        const [axis, sign] = direction.value.split(':');
        for (const side of hand.value === 'both' ? ['left', 'right'] : [hand.value]) await write(panel, slider(section, side, axis), value * Number(sign));
        const action = state.mode === 'exert' ? 'Exerting' : 'Compensating';
        state.activeValue = value;
        status.textContent = `${action} ${value} N${hand.value === 'both' ? ' per hand' : ''} · ${hand.options[hand.selectedIndex].text} · ${direction.options[direction.selectedIndex].text}. Until cleared.`;
        delete status.dataset.error;
        disclosure.textContent = `Advance Force Controls · ${value} N`;
      });
    }
    widget.querySelectorAll('[data-mode]').forEach(button => button.addEventListener('click', () => run(async () => {
      await clear(); state.mode = button.dataset.mode;
    })));
    for (const input of [hand, direction]) input.addEventListener('change', () => run(clear));
    widget.querySelector('form').addEventListener('submit', event => { event.preventDefault(); apply(custom.valueAsNumber); });
    widget.querySelector('.brace-force-clear').addEventListener('click', () => run(clear));
    const detail = widget.querySelector('.brace-force-detail-toggle');
    detail.addEventListener('click', () => detail.setAttribute('aria-expanded', String(panel.classList.toggle('brace-force-details'))));
    for (const section of [controls(panel).load, controls(panel).exert].filter(Boolean)) {
      for (const type of ['mousedown', 'keydown', 'change']) section.addEventListener(type, () => {
        if (state.busy) return;
        requestAnimationFrame(() => requestAnimationFrame(() => {
          if (state.busy) return;
          const native = controls(panel);
          const active = [...native.load.querySelectorAll('[role=slider]'),
            ...(native.mode?.checked ? native.exert.querySelectorAll('[role=slider]') : [])]
            .some(e => Number(e.getAttribute('aria-valuenow')) !== 0);
          status.textContent = active ? 'Individual axis forces active. Use Clear force to reset.' : 'No force applied.';
          state.activeValue = null;
          presetGroup.querySelectorAll('button').forEach(b => b.setAttribute('aria-pressed', 'false'));
          disclosure.textContent = active ? 'Advance Force Controls · Active' : 'Advance Force Controls';
          delete status.dataset.error;
        }));
      });
    }
    widget.addEventListener('keydown', event => { if (event.target.matches('input,select') && event.key !== 'Escape') event.stopPropagation(); });
    panel.querySelector('.mantine-Button-root')?.addEventListener('click', () => run(clear));
    widgets.set(panel, widget);
    repaint();
  }
  let pending = false;
  new MutationObserver(() => {
    if (pending) return;
    pending = true;
    requestAnimationFrame(() => { pending = false; document.querySelectorAll('.brace-controls').forEach(mount); });
  }).observe(document.body, { childList: true, subtree: true });
  document.querySelectorAll('.brace-controls').forEach(mount);
})();
