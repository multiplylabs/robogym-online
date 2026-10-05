/* Equipment gallery and the generator's upper-body pose request. */
(() => {
  'use strict';
  let selected = 'none', capable = false, statusText = 'Connecting to the gym…', phase = 'empty';
  let widget;
  const catalog = {
    dumbbells: {label:'Dumbbells', weight:'1 kg each', detail:'One in each hand · 2 kg total', icon:'<path d="M9 18h30M13 12v12M19 9v18M29 9v18M35 12v12"/>'},
    kettlebell: {label:'Kettlebell', weight:'3.5 kg', detail:'Shared handle · both hands', icon:'<path d="M17 14v-4a7 7 0 0 1 14 0v4"/><path d="M16 17a11 11 0 1 0 16 0Z"/>'},
    barbell: {label:'Barbell', weight:'2 kg', detail:'Two-hand grip · fixed width', icon:'<path d="M3 18h42M9 10v16M14 7v22M34 7v22M39 10v16"/>'}
  };
  function repaint() {
    if (!widget) return;
    widget.querySelectorAll('[data-equipment]').forEach(b => {
      b.disabled = !capable || !window.BraceGym.physics;
      b.setAttribute('aria-pressed', String(b.dataset.equipment === selected));
    });
    widget.querySelector('.brace-gym-remove').disabled = selected === 'none';
    const status = widget.querySelector('[role=status]'); status.textContent = statusText; status.dataset.phase = phase;
    widget.querySelector('.brace-gym-total').textContent = selected === 'none' ? 'No payload' : catalog[selected].detail;
  }
  window.BraceGym = {
    get selected() { return selected; },
    remove() { this.lastError = undefined; selected = 'none'; statusText = 'Empty hands. Choose a weight to begin.'; phase = 'empty'; repaint(); },
    status(text, state) { this.lastError = state === 'error' ? text : undefined; statusText = text; phase = state; repaint(); },
    async select(name) {
      if (!capable || !catalog[name]) return;
      if (selected === name) { this.remove(); return; }
      await window.braceClearForces?.();
      selected = name; this.status('Preparing grip… release the movement keys.', 'preparing');
    }
  };
  // Use the existing generator connection; context gains one backward-compatible field.
  const NativeWebSocket = window.WebSocket;
  window.WebSocket = class extends NativeWebSocket {
    constructor(...args) {
      super(...args);
      this.isGym = false;
      this.addEventListener('message', event => {
        if (typeof event.data !== 'string') return;
        let message; try { message = JSON.parse(event.data); } catch { return; }
        if (message.type !== 'hello' || !Array.isArray(message.styles)) return;
        this.isGym = true;
        capable = Object.keys(catalog).every(n => message.equipment?.includes(n));
        if (selected === 'none') statusText = capable ? 'Empty hands. Choose a weight to begin.' : 'This generator does not support equipment yet.';
        repaint();
      });
      this.addEventListener('close', () => {
        if (!this.isGym) return;
        capable = false; window.BraceGym.remove(); window.BraceGym.status('Generator disconnected. Reconnect before equipping.', 'error');
      });
    }
    send(data) {
      if (this.isGym && capable && typeof data === 'string') {
        try {
          const message = JSON.parse(data);
          if (message.type === 'context') { window.BraceGym.lastContext = {frame:message.frame,equipment:selected}; data = JSON.stringify({...message, equipment:selected}); }
        } catch { /* Leave non-JSON traffic unchanged. */ }
      }
      return super.send(data);
    }
  };
  function mount() {
    const intro = document.querySelector('.brace-panel-intro');
    if (!intro || widget?.isConnected) return;
    widget = document.createElement('section'); widget.className = 'brace-gym'; widget.setAttribute('aria-label', 'Gym equipment');
    widget.innerHTML = `<div class="brace-gym-heading"><h2>Compensate with weights</h2><span class="brace-gym-tag">GYM</span></div><p class="brace-gym-caption">Carry a real load while the robot keeps its balance.</p><div class="brace-gym-grid">${Object.entries(catalog).map(([name,item]) => `<button type="button" data-equipment="${name}" aria-label="Equip ${item.label.toLowerCase()} ${item.weight}" aria-pressed="false" disabled><svg viewBox="0 0 48 36" aria-hidden="true">${item.icon}</svg><strong>${item.label}</strong><span>${item.weight}</span></button>`).join('')}</div><div class="brace-gym-summary"><span class="brace-gym-total">No payload</span><button type="button" class="brace-gym-remove" disabled>Put down</button></div><p role="status" aria-live="polite"></p><details><summary>How carrying works</summary><p>The arms ease into a torso-relative holding pose. The legs keep your selected gait, while the controller balances the added mass and inertia.</p><p>Shared weights use a fixed two-hand grip. Fingers, slipping, dropping and equipment collisions are not simulated.</p><p>Picking up a weight clears manual forces. Use Reset to return to empty hands.</p></details>`;
    const panel = intro.closest('.brace-controls');
    const tabs = document.createElement('div'); tabs.className = 'brace-interaction-tabs';
    tabs.setAttribute('role', 'group'); tabs.setAttribute('aria-label', 'Robot interaction');
    tabs.innerHTML = '<button type="button" data-interaction="weights" aria-label="Compensate with weights" aria-pressed="true"><strong>Compensate</strong><span>Carry weights</span></button><button type="button" data-interaction="exert" aria-label="Exert force" aria-pressed="false"><strong>Exert force</strong><span>Push with the hands</span></button>';
    intro.after(tabs); tabs.after(widget);
    tabs.querySelectorAll('button').forEach(button => button.addEventListener('click', async () => {
      const exert = button.dataset.interaction === 'exert';
      if (exert) window.BraceGym.remove();
      await window.braceClearForces?.();
      panel.classList.toggle('brace-exert-view', exert);
      panel.classList.toggle('brace-forces-open', exert);
      panel.querySelector('.brace-disclosure')?.setAttribute('aria-expanded', String(exert));
      tabs.querySelectorAll('button').forEach(b => b.setAttribute('aria-pressed', String(b === button)));
      if (exert) panel.querySelector('.brace-force-actions [data-mode=exert]')?.click();
    }));
    widget.querySelectorAll('[data-equipment]').forEach(b => b.addEventListener('click', () => window.BraceGym.select(b.dataset.equipment)));
    widget.querySelector('.brace-gym-remove').addEventListener('click', () => window.BraceGym.remove());
    repaint();
  }
  let scheduled = false;
  new MutationObserver(() => {
    if (scheduled) return; scheduled = true;
    requestAnimationFrame(() => {scheduled = false; mount();});
  }).observe(document.body, {childList:true, subtree:true});
  mount();
})();
