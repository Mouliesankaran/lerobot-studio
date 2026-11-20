// ==========================================================================
// SO-101 SETUP PAGE CONTROLLER - SIMPLE, ROBUST & INTUITIVE
// ==========================================================================

const SO101Setup = (() => {
  function copyCode(btn, text) {
    if (!navigator.clipboard) {
      const ta = document.createElement('textarea');
      ta.value = text;
      document.body.appendChild(ta);
      ta.select();
      document.execCommand('copy');
      document.body.removeChild(ta);
    } else {
      navigator.clipboard.writeText(text);
    }
    const orig = btn.innerText;
    btn.innerText = 'Copied!';
    btn.classList.add('copied');
    setTimeout(() => {
      btn.innerText = orig;
      btn.classList.remove('copied');
    }, 1800);
  }

  async function updateHardwareDiagnostics() {
    const bridgeDiagEl = document.getElementById('setup-diag-bridge');
    const portDiagEl = document.getElementById('setup-diag-port');
    const liveDot = document.getElementById('setup-bridge-live-dot');
    const liveText = document.getElementById('setup-bridge-live-text');

    // Attempt to connect to relative path (if served locally) or localhost:8080
    const endpoints = [
      '/api/system/info',
      'http://localhost:8080/api/system/info',
      'http://127.0.0.1:8080/api/system/info'
    ];

    let connected = false;
    let data = null;

    for (const url of endpoints) {
      try {
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), 1200);
        const res = await fetch(url, { signal: controller.signal });
        clearTimeout(timeoutId);
        if (res.ok) {
          data = await res.json();
          connected = true;
          break;
        }
      } catch (e) {
        // Continue trying next endpoint
      }
    }

    if (connected && data) {
      if (bridgeDiagEl) {
        bridgeDiagEl.style.color = '#10b981';
        bridgeDiagEl.innerText = '● ONLINE';
      }
      if (portDiagEl && data.serial_ports && data.serial_ports.length > 0) {
        portDiagEl.innerText = `${data.serial_ports[0]} (1 Mbps)`;
      }
      if (liveDot) {
        liveDot.style.color = '#10b981';
        liveDot.innerText = '●';
      }
      if (liveText) {
        liveText.innerHTML = 'Local Hardware Bridge: <span style="color:#10b981; font-weight:800;">Connected (http://localhost:8080)</span>';
      }
    } else {
      if (bridgeDiagEl) {
        bridgeDiagEl.style.color = '#ef4444';
        bridgeDiagEl.innerText = '● OFFLINE';
      }
      if (liveDot) {
        liveDot.style.color = '#f59e0b';
        liveDot.innerText = '○';
      }
      if (liveText) {
        liveText.innerHTML = 'Waiting for Local Bridge... <span style="color:#64748b; font-weight:500;">Run Step 5 above (./run_ubuntu.sh)</span>';
      }
    }
  }

  function init() {
    // Bind all Copy buttons
    document.querySelectorAll('.rs-copy-btn').forEach(btn => {
      btn.addEventListener('click', (e) => {
        e.preventDefault();
        const codeBox = btn.closest('.rs-code-box');
        const textEl = codeBox ? codeBox.querySelector('.rs-code-text') : null;
        if (textEl) copyCode(btn, textEl.innerText.trim());
      });
    });

    // Bind "Open Robot Control Page" button
    const btnControl = document.getElementById('btn-goto-control');
    if (btnControl) {
      btnControl.addEventListener('click', () => {
        const navControl = document.querySelector('.nav-btn[data-page="control"]');
        if (navControl) {
          navControl.click();
        } else {
          // Fallback if direct page switcher
          document.querySelectorAll('.page-view').forEach(p => p.classList.remove('active'));
          const pageCtrl = document.getElementById('page-control');
          if (pageCtrl) pageCtrl.classList.add('active');
        }
      });
    }

    // Run diagnostics immediately and periodically
    updateHardwareDiagnostics();
    setInterval(updateHardwareDiagnostics, 4000);
  }

  return { init, copyCode, updateHardwareDiagnostics };
})();

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', () => SO101Setup.init());
} else {
  setTimeout(() => SO101Setup.init(), 100);
}
