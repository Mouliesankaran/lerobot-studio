/**
 * SO-101 LEADER-FOLLOWER TELEOPERATION & LEROBOT EPISODE RECORDER
 * Inspired by Piper Robot SDK / LeRobot Bilateral Teleoperation.
 *
 * Capabilities:
 * 1. Dual serial connection: Leader arm (Torque OFF for free hand guidance) & Follower arm (Torque ON).
 * 2. Independent Torque toggles for Leader and Follower.
 * 3. Integrated Camera Connection (Webcam / RealSense / Orbbec) with dual-camera view.
 * 4. 50 Hz low-latency joint mirroring from Leader to Follower.
 * 5. LeRobot demonstration recording (Start -> Pick & Place -> Save -> Replay).
 */

(function () {
  'use strict';

  const JOINTS = [
    { id: 1, name: 'Joint 1: Shoulder Pan',  key: 'shoulder_pan',  min: -84.1,  max: 84.1,  unit: '°' },
    { id: 2, name: 'Joint 2: Shoulder Lift', key: 'shoulder_lift', min: -179.9, max: 179.9, unit: '°' },
    { id: 3, name: 'Joint 3: Elbow Flex',    key: 'elbow_flex',    min: -180.0, max: 179.9, unit: '°' },
    { id: 4, name: 'Joint 4: Wrist Flex',    key: 'wrist_flex',    min: -84.3,  max: 84.3,  unit: '°' },
    { id: 5, name: 'Joint 5: Wrist Roll',    key: 'wrist_roll',    min: -180.0, max: 180.0, unit: '°' },
    { id: 6, name: 'Joint 6: Gripper',       key: 'gripper',       min: 0.0,    max: 100.0, unit: '%' }
  ];

  let statusInterval = null;
  let isMirroringActive = false;
  let isRecordingActive = false;
  let isLeaderTorqueOn = false;
  let isFollowerTorqueOn = true;
  let isCameraConnected = false;

  function init() {
    setupModeToggle();
    bindControls();
    loadSerialPorts();
    loadEpisodesList();
    initCameraControls();
  }

  function setupModeToggle() {
    const select = document.getElementById('teleop-mode-select');
    if (select) {
      if (!select.querySelector('option[value="leader"]')) {
        const opt = document.createElement('option');
        opt.value = 'leader';
        opt.textContent = '🦾 Leader-Follower & LeRobot Episode Recorder';
        select.appendChild(opt);
      }
      select.addEventListener('change', (e) => {
        applyMode(e.target.value);
      });
    }
  }

  function applyMode(mode) {
    if (mode === 'leader') {
      if (typeof switchPage === 'function') {
        switchPage('leader-teleop');
      }
    }
  }

  function startPolling() {
    if (statusInterval) clearInterval(statusInterval);
    fetchStatus();
    statusInterval = setInterval(fetchStatus, 70); // ~14 Hz UI updates
    checkAndAutoConnectCamera();
  }

  function stopPolling() {
    if (statusInterval) {
      clearInterval(statusInterval);
      statusInterval = null;
    }
  }

  function bindControls() {
    // Port controls
    const btnRefreshPorts = document.getElementById('btn-lf-refresh-ports');
    if (btnRefreshPorts) btnRefreshPorts.addEventListener('click', loadSerialPorts);

    const btnStartMirror = document.getElementById('btn-lf-start-mirror');
    if (btnStartMirror) btnStartMirror.addEventListener('click', toggleMirroring);

    // Torque controls
    const btnLeaderTorque = document.getElementById('btn-lf-leader-torque');
    if (btnLeaderTorque) btnLeaderTorque.addEventListener('click', toggleLeaderTorque);

    const btnFollowerTorque = document.getElementById('btn-lf-follower-torque');
    if (btnFollowerTorque) btnFollowerTorque.addEventListener('click', toggleFollowerTorque);

    // Camera controls
    const btnCamConnect = document.getElementById('btn-lf-cam-connect');
    if (btnCamConnect) btnCamConnect.addEventListener('click', connectCamera);

    const btnCamDisconnect = document.getElementById('btn-lf-cam-disconnect');
    if (btnCamDisconnect) btnCamDisconnect.addEventListener('click', disconnectCamera);

    // Episode recording controls
    const btnRecStart = document.getElementById('btn-lf-rec-start');
    if (btnRecStart) btnRecStart.addEventListener('click', startEpisode);

    const btnRecSave = document.getElementById('btn-lf-rec-save');
    if (btnRecSave) btnRecSave.addEventListener('click', stopAndSaveEpisode);

    const btnRecDiscard = document.getElementById('btn-lf-rec-discard');
    if (btnRecDiscard) btnRecDiscard.addEventListener('click', discardEpisode);

    const btnRefreshEpisodes = document.getElementById('btn-lf-refresh-episodes');
    if (btnRefreshEpisodes) btnRefreshEpisodes.addEventListener('click', loadEpisodesList);
  }

  // -------------------------------------------------------------------------
  // Serial Ports Management
  // -------------------------------------------------------------------------

  async function loadSerialPorts() {
    try {
      const res = await fetch('/api/teleop/leader_follower/ports');
      const data = await res.json();
      if (!data.success) return;

      const leaderSelect = document.getElementById('lf-leader-port-select');
      const followerSelect = document.getElementById('lf-follower-port-select');
      const ports = data.ports || [];

      function populate(sel, currentVal, preferredFallback) {
        if (!sel) return;
        sel.innerHTML = '';
        if (ports.length === 0) {
          sel.innerHTML = `<option value="simulation">simulation (Virtual Arm)</option>`;
          return;
        }

        ports.forEach((p) => {
          const opt = document.createElement('option');
          opt.value = p;
          opt.textContent = p === 'simulation' ? 'simulation (Virtual Arm Mode)' : p;
          if (p === currentVal || (!currentVal && p === preferredFallback)) {
            opt.selected = true;
          }
          sel.appendChild(opt);
        });
      }

      populate(leaderSelect, data.leader_port, '/dev/ttyACM1');
      populate(followerSelect, data.follower_port, '/dev/ttyACM0');
    } catch (err) {
      console.warn('Failed to load leader-follower ports:', err);
    }
  }

  // -------------------------------------------------------------------------
  // Torque Control (Leader Free Guiding vs Follower Holding)
  // -------------------------------------------------------------------------

  async function toggleLeaderTorque() {
    const nextState = !isLeaderTorqueOn;
    try {
      const res = await fetch('/api/teleop/leader/torque', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ enable: nextState })
      });
      const data = await res.json();
      if (data.success) {
        isLeaderTorqueOn = nextState;
        updateTorqueButtons();
      }
    } catch (err) {
      alert('Error updating Leader torque: ' + err.message);
    }
  }

  async function toggleFollowerTorque() {
    const nextState = !isFollowerTorqueOn;
    try {
      const res = await fetch('/api/teleop/follower/torque', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ enable: nextState })
      });
      const data = await res.json();
      if (data.success) {
        isFollowerTorqueOn = nextState;
        updateTorqueButtons();
      }
    } catch (err) {
      alert('Error updating Follower torque: ' + err.message);
    }
  }

  function updateTorqueButtons() {
    const btnL = document.getElementById('btn-lf-leader-torque');
    if (btnL) {
      if (isLeaderTorqueOn) {
        btnL.className = 'btn btn-danger';
        btnL.innerHTML = '⚡ Leader Torque: ON (Locked)';
        btnL.title = 'Click to disable torque so you can guide the arm by hand';
      } else {
        btnL.className = 'btn btn-outline';
        btnL.innerHTML = '⚡ Leader Torque: OFF (Hand Guiding)';
        btnL.title = 'Torque is OFF. You can freely guide this arm with your hands';
      }
    }

    const btnF = document.getElementById('btn-lf-follower-torque');
    if (btnF) {
      if (isFollowerTorqueOn) {
        btnF.className = 'btn btn-success';
        btnF.innerHTML = '⚡ Follower Torque: ACTIVE';
      } else {
        btnF.className = 'btn btn-warning';
        btnF.innerHTML = '⚡ Follower Torque: OFF';
      }
    }
  }

  // -------------------------------------------------------------------------
  // Camera Connection & Live Stream Management
  // -------------------------------------------------------------------------

  async function initCameraControls() {
    try {
      const res = await fetch('/api/camera/devices?refresh=1');
      const data = await res.json();
      const sel = document.getElementById('lf-cam-device-select');
      if (sel && data.devices && data.devices.length > 0) {
        sel.innerHTML = '';
        data.devices.forEach((d) => {
          const opt = document.createElement('option');
          opt.value = d.id;
          opt.textContent = d.name || d.id;
          sel.appendChild(opt);
        });
      }
      checkAndAutoConnectCamera();
    } catch (e) {
      console.warn('Camera device query failed:', e);
    }
  }

  async function checkAndAutoConnectCamera() {
    try {
      const statusRes = await fetch('/api/camera/status');
      const statusData = await statusRes.json();
      if (statusData.connected) {
        setCameraUI(true, statusData.device_id);
      } else {
        // Auto connect if disconnected
        connectCamera();
      }
    } catch (e) {}
  }

  async function connectCamera() {
    const sel = document.getElementById('lf-cam-device-select');
    const devId = (sel && sel.value) ? sel.value : '/dev/video0';
    const btnConnect = document.getElementById('btn-lf-cam-connect');
    const btnDisconnect = document.getElementById('btn-lf-cam-disconnect');

    if (btnConnect) {
      btnConnect.disabled = true;
      btnConnect.textContent = 'Connecting...';
    }

    try {
      const res = await fetch('/api/camera/connect', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          device_id: devId,
          camera_type: 'usb',
          width: 640,
          height: 480,
          fps: 30
        })
      });
      const data = await res.json();
      if (data.success) {
        setCameraUI(true, devId);
      } else {
        setCameraUI(false);
      }
    } catch (err) {
      setCameraUI(false);
    } finally {
      if (btnConnect) {
        btnConnect.disabled = isCameraConnected;
        btnConnect.textContent = '▶ Connect Camera';
      }
      if (btnDisconnect) btnDisconnect.disabled = !isCameraConnected;
    }
  }

  async function disconnectCamera() {
    try {
      await fetch('/api/camera/disconnect', { method: 'POST' });
    } catch (e) {}
    setCameraUI(false);
  }

  function setCameraUI(online, devId) {
    isCameraConnected = online;
    const badge = document.getElementById('lf-badge-camera');
    const btnConnect = document.getElementById('btn-lf-cam-connect');
    const btnDisconnect = document.getElementById('btn-lf-cam-disconnect');

    if (badge) {
      badge.className = online ? 'lf-pill success' : 'lf-pill warning';
      badge.innerHTML = online ? `📷 Camera: LIVE (${devId || 'USB'})` : '📷 Camera: Standby';
    }

    if (btnConnect) btnConnect.disabled = online;
    if (btnDisconnect) btnDisconnect.disabled = !online;

    // Refresh image streams with cache buster if online
    if (online) {
      const topImg = document.getElementById('lf-top-cam-img');
      const wristImg = document.getElementById('lf-wrist-cam-img');
      const now = Date.now();
      if (topImg) topImg.src = `/api/camera/stream?mode=rgb&t=${now}`;
      if (wristImg) wristImg.src = `/api/camera/stream?mode=depth&t=${now}`;
    }
  }

  // -------------------------------------------------------------------------
  // Bilateral Mirroring Toggle
  // -------------------------------------------------------------------------

  async function toggleMirroring() {
    const btn = document.getElementById('btn-lf-start-mirror');
    if (isMirroringActive) {
      try {
        btn.disabled = true;
        btn.textContent = 'Stopping...';
        await fetch('/api/teleop/leader_follower/stop', { method: 'POST' });
        isMirroringActive = false;
        updateMirrorButton(false);
      } catch (err) {
        alert('Failed to stop mirroring: ' + err.message);
      } finally {
        btn.disabled = false;
      }
    } else {
      const leaderSelect = document.getElementById('lf-leader-port-select');
      const followerSelect = document.getElementById('lf-follower-port-select');
      const leaderPort = leaderSelect ? leaderSelect.value : '';
      const followerPort = followerSelect ? followerSelect.value : '';

      if (!leaderPort) {
        alert('Please select a serial port for the Leader Arm.');
        return;
      }

      try {
        btn.disabled = true;
        btn.textContent = 'Connecting...';
        const res = await fetch('/api/teleop/leader_follower/start', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ leader_port: leaderPort, follower_port: followerPort })
        });
        const data = await res.json();
        if (data.success) {
          isMirroringActive = true;
          updateMirrorButton(true);
        } else {
          alert('Failed to start mirroring:\n' + (data.message || 'Unknown error'));
          updateMirrorButton(false);
        }
      } catch (err) {
        alert('Mirroring connection failed: ' + err.message);
        updateMirrorButton(false);
      } finally {
        btn.disabled = false;
      }
    }
  }

  function updateMirrorButton(active) {
    const btn = document.getElementById('btn-lf-start-mirror');
    if (!btn) return;
    if (active) {
      btn.innerHTML = '⏹ Stop Mirroring';
      btn.className = 'btn btn-danger';
    } else {
      btn.innerHTML = '▶ Start Mirroring';
      btn.className = 'btn btn-primary';
    }
  }

  // -------------------------------------------------------------------------
  // Status & Telemetry Loop
  // -------------------------------------------------------------------------

  async function fetchStatus() {
    try {
      const res = await fetch('/api/teleop/leader_follower/status');
      const data = await res.json();

      isMirroringActive = !!data.is_mirroring;
      updateMirrorButton(isMirroringActive);

      // Torque status
      isLeaderTorqueOn = !!data.leader_torque_enabled;
      updateTorqueButtons();

      // HUD Badges
      const leaderBadge = document.getElementById('lf-badge-leader');
      if (leaderBadge) {
        leaderBadge.className = data.leader_connected ? 'lf-pill success' : 'lf-pill warning';
        leaderBadge.innerHTML = data.leader_connected
          ? `🦾 Leader: Connected (${data.leader_port || 'Active'})`
          : `🦾 Leader: Disconnected`;
      }

      const followerBadge = document.getElementById('lf-badge-follower');
      if (followerBadge) {
        followerBadge.className = data.follower_connected ? 'lf-pill success' : 'lf-pill warning';
        followerBadge.innerHTML = data.follower_connected
          ? `🦾 Follower: Connected (${data.follower_port || 'Active'})`
          : `🦾 Follower: Disconnected`;
      }

      const syncRateEl = document.getElementById('lf-sync-rate');
      if (syncRateEl) {
        syncRateEl.textContent = `${data.sync_rate_hz || 0.0} Hz`;
      }

      const latencyEl = document.getElementById('lf-latency');
      if (latencyEl) {
        latencyEl.textContent = `${data.latency_ms || 0.0} ms`;
      }

      // Joint comparison bars
      updateJointCards(data.leader_angles || {}, data.follower_angles || {});

      // Recorder HUD
      updateRecorderHUD(data.recording || {});
    } catch (err) {}
  }

  function updateJointCards(leaderAngles, followerAngles) {
    JOINTS.forEach((j) => {
      const lDeg = parseFloat(leaderAngles[j.id] || 0.0);
      const fDeg = parseFloat(followerAngles[j.id] || 0.0);
      const err = Math.abs(lDeg - fDeg);

      const valLeader = document.getElementById(`lf-val-l-${j.id}`);
      if (valLeader) valLeader.textContent = `${lDeg.toFixed(1)}${j.unit}`;

      const valFollower = document.getElementById(`lf-val-f-${j.id}`);
      if (valFollower) valFollower.textContent = `${fDeg.toFixed(1)}${j.unit}`;

      const errBadge = document.getElementById(`lf-err-${j.id}`);
      if (errBadge) {
        errBadge.textContent = `Δ ${err.toFixed(1)}${j.unit}`;
        if (err > 5.0) errBadge.className = 'lf-val-error high';
        else if (err > 2.0) errBadge.className = 'lf-val-error warn';
        else errBadge.className = 'lf-val-error';
      }

      const span = Math.max(1, j.max - j.min);
      const lPct = Math.max(0, Math.min(100, ((lDeg - j.min) / span) * 100));
      const fPct = Math.max(0, Math.min(100, ((fDeg - j.min) / span) * 100));

      const barL = document.getElementById(`lf-bar-l-${j.id}`);
      if (barL) barL.style.width = `${lPct}%`;

      const barF = document.getElementById(`lf-bar-f-${j.id}`);
      if (barF) barF.style.width = `${fPct}%`;
    });
  }

  // -------------------------------------------------------------------------
  // LeRobot Episode Recorder Controls
  // -------------------------------------------------------------------------

  function updateRecorderHUD(rec) {
    isRecordingActive = !!rec.is_recording;

    const badge = document.getElementById('lf-rec-status-badge');
    if (badge) {
      if (isRecordingActive) {
        badge.className = 'lf-rec-badge recording';
        badge.innerHTML = `🔴 RECORDING LIVE`;
      } else {
        badge.className = 'lf-rec-badge';
        badge.innerHTML = `⚪ STANDBY`;
      }
    }

    const epNum = document.getElementById('lf-rec-ep-num');
    if (epNum) epNum.textContent = `Episode #${rec.current_episode || 1}`;

    const timer = document.getElementById('lf-rec-timer');
    if (timer) timer.textContent = `${(rec.elapsed_seconds || 0.0).toFixed(1)}s`;

    const frames = document.getElementById('lf-rec-frames');
    if (frames) frames.textContent = `${rec.current_frames || 0} frames`;

    const btnStart = document.getElementById('btn-lf-rec-start');
    const btnSave = document.getElementById('btn-lf-rec-save');
    const btnDiscard = document.getElementById('btn-lf-rec-discard');

    if (btnStart) btnStart.disabled = isRecordingActive;
    if (btnSave) btnSave.disabled = !isRecordingActive;
    if (btnDiscard) btnDiscard.disabled = !isRecordingActive;
  }

  async function startEpisode() {
    const taskInput = document.getElementById('lf-task-input');
    const task = taskInput ? taskInput.value : 'Pick cube and place in target bowl';

    try {
      const res = await fetch('/api/teleop/episode/start', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ task })
      });
      const data = await res.json();
      if (!data.success) {
        alert('Could not start episode: ' + data.message);
      }
    } catch (err) {
      alert('Error starting episode: ' + err.message);
    }
  }

  async function stopAndSaveEpisode() {
    try {
      const btn = document.getElementById('btn-lf-rec-save');
      if (btn) btn.disabled = true;
      const res = await fetch('/api/teleop/episode/stop', { method: 'POST' });
      const data = await res.json();
      if (data.success) {
        loadEpisodesList();
      } else {
        alert('Could not save episode: ' + data.message);
      }
    } catch (err) {
      alert('Error saving episode: ' + err.message);
    }
  }

  async function discardEpisode() {
    if (!confirm('Discard this demonstration episode without saving?')) return;
    try {
      await fetch('/api/teleop/episode/discard', { method: 'POST' });
    } catch (err) {
      alert('Error discarding episode: ' + err.message);
    }
  }

  async function loadEpisodesList() {
    const tbody = document.getElementById('lf-episodes-tbody');
    if (!tbody) return;

    try {
      const res = await fetch('/api/teleop/episode/list');
      const data = await res.json();
      if (!data.success) return;

      const totalSpan = document.getElementById('lf-total-episodes-count');
      if (totalSpan) totalSpan.textContent = `(${data.total_episodes || 0} episodes saved)`;

      const episodes = data.episodes || [];
      if (episodes.length === 0) {
        tbody.innerHTML = `<tr><td colspan="7" style="text-align:center; color:var(--text-muted); padding:14px;">No episodes recorded yet. Click "Start Episode" above to collect demonstrations!</td></tr>`;
        return;
      }

      tbody.innerHTML = '';
      episodes.forEach((ep) => {
        const tr = document.createElement('tr');
        const dt = ep.created_at ? ep.created_at.replace('T', ' ').slice(0, 19) : '-';
        tr.innerHTML = `
          <td style="font-weight:700; color:var(--cyan-accent);">#${ep.episode_index}</td>
          <td>${ep.task || 'Cube pick & place'}</td>
          <td style="font-family:monospace;">${ep.total_frames || 0}</td>
          <td style="font-family:monospace;">${(ep.duration_seconds || 0).toFixed(1)}s</td>
          <td style="font-family:monospace;">${ep.fps || 0} Hz</td>
          <td style="color:var(--text-muted); font-size:0.7rem;">${dt}</td>
          <td>
            <button class="lf-btn-mini btn-replay" data-ep="${ep.episode_index}" title="Replay trajectory on follower robot">▶ Replay</button>
          </td>
        `;
        tbody.appendChild(tr);
      });

      tbody.querySelectorAll('.btn-replay').forEach((btn) => {
        btn.addEventListener('click', (e) => {
          const ep = parseInt(e.target.getAttribute('data-ep'), 10);
          replayEpisode(ep);
        });
      });
    } catch (err) {
      console.warn('Failed to load episode list:', err);
    }
  }

  async function replayEpisode(epIdx) {
    if (!confirm(`Replay Episode #${epIdx} on the physical follower robot?`)) return;
    try {
      const res = await fetch('/api/teleop/episode/replay', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ episode_index: epIdx, speed: 1.0 })
      });
      const data = await res.json();
      if (!data.success) {
        alert('Replay failed: ' + data.message);
      }
    } catch (err) {
      alert('Error triggering replay: ' + err.message);
    }
  }

  // Export & Initialize
  window.SO101LeaderTeleop = {
    init,
    startMirroring: toggleMirroring,
    startEpisode,
    stopAndSaveEpisode,
    discardEpisode,
    startPolling,
    stopPolling,
    connectCamera,
    disconnectCamera,
    toggleLeaderTorque,
    toggleFollowerTorque
  };

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
