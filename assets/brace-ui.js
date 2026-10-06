/* Presentation only: keep simulator controls, keyboard handlers and policy state native. */
(() => {
  'use strict';
  window.BraceReference = {mode:'body'};
  const header = document.createElement('header');
  header.className = 'brace-header';
  header.innerHTML = `<h1><a class="brace-brand" href="https://multiplylabs.github.io/brace/" target="_blank" rel="noopener">BRACE<span>Interactive demo</span></a></h1><div class="brace-header-actions"><label class="brace-reference-view">Reference<select aria-label="Reference view"><option value="body">Body-aligned pose</option><option value="world">World trajectory</option></select></label><button type="button" id="brace-help-toggle" aria-expanded="false" aria-controls="brace-guide">Annotations</button><a href="https://multiplylabs.github.io/brace/" target="_blank" rel="noopener">Research ↗</a></div>`;
  document.body.append(header);
  const guide = document.createElement('aside');
  guide.id = 'brace-guide';
  guide.className = 'brace-guide';
  guide.hidden = true;
  guide.setAttribute('aria-label', 'Scene annotations');
  guide.innerHTML = `<div class="brace-eyebrow">Scene Guide</div><h2>What You’re Seeing</h2><dl><dt><i class="brace-dot robot"></i>Solid robot</dt><dd>The simulated G1 responding to the controller.</dd><dt><i class="brace-dot reference"></i>Green reference</dt><dd>The selected MotionBricks motion with the active holding pose. The default body-aligned overlay removes global position offset, including terrain height, so you can compare the pose. It preserves heading differences. Choose World trajectory to inspect the original path and position offset; uncheck “Show reference” to hide it.</dd><dt><i class="brace-dot braced"></i>Red braced motion</dt><dd>BRACE adjusts the reference to exert the requested hand force.</dd><dt>Spring force gauge</dt><dd>In Exert mode, the hand presses a moving pad. Blue marks the effective force target; amber shows the simulated reaction. The live tile shows their difference. Release movement keys and let the gauge settle for a steady reading.</dd><dt><i class="brace-dot left"></i><i class="brace-dot right"></i>Hand load arrows</dt><dd>Orange is the left hand; blue is the right. Direction and length show the applied load.</dd></dl><p><strong>Try a slope.</strong> Enable “Slope ahead,” then hold W to walk across it. Release the key to stop.</p><p><strong>Pick up a weight.</strong> Choose dumbbells, a kettlebell or a barbell in the equipment gallery. To push, choose the visible “Exert force” action, and click a force preset or enter a custom value. Use Clear force to remove it.</p><p class="brace-note">Choose gym equipment to add physical mass and inertia. Shared weights use an ideal two-hand grip; fingers and dropping are not simulated.</p>`;
  document.body.append(guide);
  header.querySelector('[aria-label="Reference view"]').addEventListener('change',event => {window.BraceReference.mode=event.target.value;});
  const help = header.querySelector('button');
  help.addEventListener('click', () => {
    guide.hidden = !guide.hidden;
    help.setAttribute('aria-expanded', String(!guide.hidden));
  });
  window.addEventListener('keydown', event => {
    if (event.key === 'Escape' && !guide.hidden) {
      guide.hidden = true;
      help.setAttribute('aria-expanded', 'false');
      help.focus();
    }
  });

  let catalog;
  fetch(new URL('assets/config.json', document.baseURI))
    .then(r => r.ok ? r : fetch(new URL('../assets/config.json', document.baseURI)))
    .then(r => r.ok ? r.json() : null)
    .then(value => { catalog = value; enhance(); }).catch(() => {});
  const panels = new WeakSet();
  const labels = { slow_walk: 'Slow walk', stealth: 'Stealth', object_carrying: 'Object carrying', careful: 'Careful' };
  const notes = { slow_walk: 'Relaxed steps · 0.5 m/s target', stealth: 'Low, quiet gait · 0.8 m/s target', object_carrying: 'Arms held forward · 0.8 m/s target', careful: 'Deliberate steps · 0.8 m/s target' };
  function enhance() {
    const input = document.getElementById('policy-select');
    const panel = input?.closest('.mantine-Paper-root');
    if (panel) {
      panel.classList.add('brace-controls');
      if (!panels.has(panel)) {
        panels.add(panel);
        const title = document.createElement('div');
        title.className = 'brace-panel-intro';
        title.innerHTML = '<div class="brace-eyebrow">Explore BRACE</div><p>Choose weights to compensate a load, or Exert force to push. Pick a walking style and hold the direction keys to move.</p>';
        const content = panel.querySelector('.mantine-ScrollArea-content > div > div');
        if (content) {
          content.prepend(title);
          const expand = document.createElement('button');
          expand.type = 'button';
          expand.className = 'brace-disclosure';
          expand.textContent = 'Advance Force Controls';
          expand.setAttribute('aria-label', 'Advance Force Controls');
          expand.setAttribute('aria-expanded', 'false');
          expand.addEventListener('click', () => {
            const opened = panel.classList.toggle('brace-forces-open');
            expand.setAttribute('aria-expanded', String(opened));
          });
          const reset = content.querySelector('.mantine-Button-root');
          content.insertBefore(expand, content.querySelector('.mantine-Paper-root[data-with-border]') ?? reset?.parentElement ?? null);
          const advanced = document.createElement('button');
          advanced.type = 'button';
          advanced.className = 'brace-advanced';
          advanced.textContent = 'Display & model settings';
          advanced.setAttribute('aria-expanded', 'false');
          advanced.addEventListener('click', () => {
            const opened = panel.classList.toggle('brace-advanced-open');
            advanced.setAttribute('aria-expanded', String(opened));
          });
          content.append(advanced);
        }
      }
      // Keep selectors available for multi-policy builds and in advanced settings.
      const project = catalog?.projects?.find(p => p.scenes?.some(s => s.name === document.getElementById('scene-select')?.value));
      const scene = project?.scenes?.find(s => s.name === document.getElementById('scene-select')?.value);
      const policy = scene?.policies?.find(p => p.name === input.value);
      for (const [id, count] of [['scene-select', project?.scenes?.length], ['policy-select', scene?.policies?.length], ['motion-select', policy?.motions?.length]]) {
        const selector = document.getElementById(id);
        const row = selector?.closest('.mantine-Flex-root')?.parentElement;
        row?.classList.toggle('brace-single-choice', count === 1);
      }
      panel.querySelectorAll('.mantine-Paper-root[data-with-border]').forEach(section => {
        const toggle = section.firstElementChild;
        const name = toggle?.textContent.trim();
        section.classList.add('brace-section');
        section.classList.toggle('brace-force-section', name === 'Hand Force' || name === 'Exert');
        section.classList.toggle('brace-debug-section', name === 'Debug Viz');
        if (toggle && !toggle.hasAttribute('role')) {
          toggle.setAttribute('role', 'button');
          toggle.tabIndex = 0;
          toggle.addEventListener('keydown', e => {
            if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); toggle.click(); }
          });
          toggle.addEventListener('click', () => setTimeout(enhance, 220));
        }
        toggle?.setAttribute('aria-expanded', String(section.children[1]?.getAttribute('aria-hidden') === 'false'));
      });
      panel.querySelectorAll('[role="slider"]').forEach(slider => {
        const row = slider.closest('.mantine-Slider-root')?.parentElement?.parentElement;
        const label = row?.querySelector('p')?.textContent;
        if (label) slider.setAttribute('aria-label', label);
      });
    }
    // LiveMotionSource creates this HUD outside React; preserve its click and key bindings.
    for (const hud of document.body.children) {
      if (hud.style.position !== 'fixed' || hud.style.zIndex !== '40') continue;
      hud.classList.add('brace-hud');
      for (const card of hud.children) {
        const kind = card.firstElementChild?.textContent;
        if (kind === 'steer') {
          card.classList.add('brace-steering');
          card.querySelectorAll('div:nth-child(2) > div > span').forEach(cap => {
            const key = cap.textContent.toLowerCase();
            const names = { w: 'Walk forward', s: 'Walk backward', a: 'Step left', d: 'Step right', q: 'Turn left', e: 'Turn right' };
            if (!names[key] || cap.dataset.braceReady) return;
            cap.dataset.braceReady = 'true';
            cap.setAttribute('role', 'button');
            cap.setAttribute('aria-label', names[key]);
            cap.title = `${names[key]} · hold to move`;
            cap.tabIndex = 0;
            const send = type => window.dispatchEvent(new KeyboardEvent(type, { key }));
            cap.addEventListener('pointerdown', event => {
              event.preventDefault();
              cap.setPointerCapture(event.pointerId);
              send('keydown');
            });
            for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) cap.addEventListener(type, () => send('keyup'));
            cap.addEventListener('keydown', event => {
              if ((event.key === 'Enter' || event.key === ' ') && !event.repeat) {
                event.preventDefault(); event.stopPropagation(); send('keydown');
              }
            });
            cap.addEventListener('keyup', event => {
              if (event.key === 'Enter' || event.key === ' ') {
                event.preventDefault(); event.stopPropagation(); send('keyup');
              }
            });
            cap.addEventListener('blur', () => send('keyup'));
          });
        }
        if (kind?.startsWith('style')) {
          card.classList.add('brace-styles');
          [...card.children].slice(1).forEach(row => {
            const raw = row.lastChild?.textContent.trim().replaceAll(' ', '_');
            row.classList.add('brace-style');
            row.classList.toggle('is-active', row.firstElementChild?.textContent === '●');
            row.setAttribute('role', 'button');
            row.tabIndex = 0;
            row.setAttribute('aria-pressed', String(row.classList.contains('is-active')));
            row.setAttribute('aria-label', labels[raw] ?? raw);
            row.title = notes[raw] ?? '';
            if (!row.dataset.braceReady) {
              row.dataset.braceReady = 'true';
              row.addEventListener('keydown', e => {
                if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); row.click(); }
              });
            }
          });
        }
      }
    }
  }
  let scheduled = false;
  new MutationObserver(() => {
    if (scheduled) return;
    scheduled = true;
    requestAnimationFrame(() => { scheduled = false; enhance(); });
  }).observe(document.body, { childList: true, subtree: true });
  enhance();
})();
