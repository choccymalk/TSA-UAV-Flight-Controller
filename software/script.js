function switchMenu(name) {
  document.querySelectorAll('.menu-content').forEach(el => el.classList.remove('active'));
  document.querySelectorAll('.menu-tab').forEach(el => el.classList.remove('active'));
  document.getElementById(name).classList.add('active');
  event.target.classList.add('active');
}

function loadVideoFeed() {
  videoFeed.innerHTML = '<img src="http://' + window.location.hostname + ':8010/stream" alt="Flight Controller Video" width="1280" height="800" onerror="this.parentElement.innerHTML=\'<div class=\'video-placeholder\'><p>Failed to load video stream</p><p style=\'font-size:12px;color:var(--muted)\'>http://' + window.location.hostname + ':8010/stream</p></div>\'" />';
}

function requestTelemetry() {
  if (window.requestTelemetry) window.requestTelemetry();
}

function toggleAutoTelemetry() {
  if (window.toggleAutoTelemetry) window.toggleAutoTelemetry();
}

function sendManualData() {
  const data = document.getElementById('sendDataBox').value;
  if (data && window.sendData) window.sendData(data);
}

// Update connection status
function updateConnectionStatus(connected) {
  const indicator = document.getElementById('connIndicator');
  const status = document.getElementById('connStatus');
  if (connected) {
      indicator.className = 'status-indicator connected';
      status.textContent = 'Connected';
  } else {
      indicator.className = 'status-indicator disconnected';
      status.textContent = 'Disconnected';
  }
}

// Update gamepad status
function updateGamepadStatus(connected) {
  const indicator = document.getElementById('gpIndicator');
  const status = document.getElementById('gpStatus');
  if (connected) {
      indicator.className = 'status-indicator connected';
      status.textContent = 'Connected';
  } else {
      indicator.className = 'status-indicator disconnected';
      status.textContent = 'Not connected';
  }
}

let ws = null;
let autoTelemetryEnabled = false;
let commandSendInterval = null;

function connectWebSocket() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = protocol + '//' + window.location.hostname + ':8009';
  ws = new WebSocket(wsUrl);

  ws.onopen = function () {
      console.log('WebSocket connected');
      updateConnectionStatus(true);  // Now properly updates the connection indicator

      // Start sending control commands at safe refresh rate
      if (!commandSendInterval) {
          commandSendInterval = setInterval(sendControlCommands, 25); // 40Hz - but server adds 5ms delay
      }
  };


  ws.onmessage = function (event) {
      const data = event.data;
      document.getElementById('parsedDataBox').innerText = data;
      const parsed = parseDataFurther(data);
      updateDisplayValues(parsed);
  };

  ws.onerror = function (error) {
      console.error('WebSocket error:', error);
      updateConnectionStatus(false);  // Was trying to update non-existent 'status' element
  };


  ws.onclose = function () {
      console.log('WebSocket disconnected');
      updateConnectionStatus(false);  // Was trying to update non-existent 'status' element
      if (commandSendInterval) {
          clearInterval(commandSendInterval);
          commandSendInterval = null;
      }
      // Attempt to reconnect after 3 seconds
      setTimeout(connectWebSocket, 3000);
  };
}

function sendControlCommands() {
  if (!ws || ws.readyState !== WebSocket.OPEN) {
      return;
  }

  const throttle = document.getElementById('userThrottleSlider').value;
  const pitch = document.getElementById('userPitchSlider').value;
  const roll = document.getElementById('userRollSlider').value;
  const yaw = document.getElementById('userYawSlider').value;

  const command = ".t" + throttle + ";" + ".p" + pitch + ";" + ".r" + roll + ";" + ".y" + yaw + ";";
  ws.send('send:' + command);
}

function sendManualData() {
  if (!ws || ws.readyState !== WebSocket.OPEN) {
      alert('WebSocket not connected');
      return;
  }

  const dataToSend = document.getElementById('sendDataBox').value;
  if (dataToSend.trim() !== '') {
      ws.send('send:' + dataToSend);
      console.log('Manual data sent: ' + dataToSend);
  }
}

function requestTelemetry() {
  if (!ws || ws.readyState !== WebSocket.OPEN) {
      alert('WebSocket not connected');
      return;
  }
  ws.send('get_data');
  console.log('Telemetry requested');
}

function toggleAutoTelemetry() {
  autoTelemetryEnabled = !autoTelemetryEnabled;
  if (autoTelemetryEnabled) {
      console.log('Auto telemetry enabled');
      autoTelemetryLoop();
  } else {
      console.log('Auto telemetry disabled');
  }
}

function autoTelemetryLoop() {
  if (autoTelemetryEnabled && ws && ws.readyState === WebSocket.OPEN) {
      ws.send('get_data');
  }
  if (autoTelemetryEnabled) {
      setTimeout(autoTelemetryLoop, 500); // Request telemetry every 500 ms
  }
}

function updateDisplayValues(parsed) {
  // Telemetry display values (these are divs, not inputs - use textContent)
  document.getElementById('throttleValue').textContent = parsed.throttle.toFixed(2);
  document.getElementById('actualPitchValue').textContent = parsed.actualPitch.toFixed(2) + '°';
  document.getElementById('actualRollValue').textContent = parsed.actualRoll.toFixed(2) + '°';
  document.getElementById('actualYawValue').textContent = parsed.actualYaw.toFixed(2) + '°';
  document.getElementById('targetPitchValue').textContent = parsed.targetPitch.toFixed(2) + '°';
  document.getElementById('targetRollValue').textContent = parsed.targetRoll.toFixed(2) + '°';
  document.getElementById('targetYawValue').textContent = parsed.targetYaw.toFixed(2) + '°';
  document.getElementById('batteryVoltageValue').textContent = parsed.batteryVoltage.toFixed(1);
}

function updateUserThrottle() {
  document.getElementById('userThrottleValue').innerText = document.getElementById('userThrottleSlider').value;
}

function updateUserPitch() {
  document.getElementById('userPitchValue').innerText = document.getElementById('userPitchSlider').value;
}

function updateUserRoll() {
  document.getElementById('userRollValue').innerText = document.getElementById('userRollSlider').value;
}

function updateUserYaw() {
  document.getElementById('userYawValue').innerText = document.getElementById('userYawSlider').value;
}

function parseDataFurther(data) {
  const dataArray = data.split('|');
  return {
      throttle: parseFloat(dataArray[0]) || 0,
      actualPitch: parseFloat(dataArray[1]) || 0,
      actualRoll: parseFloat(dataArray[2]) || 0,
      actualYaw: parseFloat(dataArray[3]) || 0,
      targetPitch: parseFloat(dataArray[4]) || 0,
      targetRoll: parseFloat(dataArray[5]) || 0,
      targetYaw: parseFloat(dataArray[6]) || 0,
      pidOutputPitch: parseFloat(dataArray[7]) || 0,
      pidOutputRoll: parseFloat(dataArray[8]) || 0,
      pidOutputYaw: parseFloat(dataArray[9]) || 0,
      batteryVoltage: parseFloat(dataArray[10]) || 0  // Fixed: was "0." with trailing dot
  };
}

// Initialize WebSocket connection on page load
window.addEventListener('load', function () {
  setTimeout(connectWebSocket, 1000);
});

// CORRECTED GAMEPAD CODE SECTION
const doc = id => document.getElementById(id);

let gpIndex = null;
let pollingGamepad = true;
let pollHandle = null;
let lastAxes = [];
let listeningFor = null; // { control, startedAt }

const state = {
  map: { yaw: 0, pitch: 1, roll: 2, throttle: 3 },
  invert: { pitch: false, throttle: false },
  zeroOffsets: { yaw: 0, pitch: 0, roll: 0, throttle: 0 }
};

// Elements for mapping UI
const selects = {
  roll: doc('mapRoll'),
  pitch: doc('mapPitch'),
  yaw: doc('mapYaw'),
  throttle: doc('mapThrottle')
};
const learnBtns = {
  roll: doc('learnRoll'),
  pitch: doc('learnPitch'),
  yaw: doc('learnYaw'),
  throttle: doc('learnThrottle')
};
const invertPitch = doc('invertPitch');
const invertThrottle = doc('invertThrottle');
const autoMapBtn = doc('autoMap');
const calibrateZeroBtn = doc('calibrateZero');
const connectHintBtn = doc('connectHint');
const stopPollingBtn = doc('stopPolling');

function fxClamp(v, a, b) { return Math.max(a, Math.min(b, v)); }

// FIX: Use the proper status update function instead of directly manipulating DOM
function setGamepadStatus(text) {
  doc('gpStatusDetail').textContent = text;  // Changed from 'gpStatus' to 'gpStatusDetail'
  updateGamepadStatus(gpIndex !== null);  // Update the visual indicator too
}

function populateAxesSelects(count) {
  [selects.roll, selects.pitch, selects.yaw, selects.throttle].forEach(s => {
      const prev = s.value;
      s.innerHTML = '';
      for (let i = 0; i < count; i++) {
          const opt = document.createElement('option');
          opt.value = String(i);
          opt.textContent = `Axis ${i}`;
          s.appendChild(opt);
      }
      if (prev !== '' && +prev < count) s.value = prev;
  });
}

// Seed selects with default 6 axes
populateAxesSelects(6);

// Gamepad connect/disconnect event listeners
window.addEventListener("gamepadconnected", (e) => {
  gpIndex = e.gamepad.index;
  setGamepadStatus(`Gamepad connected: ${e.gamepad.id} — ${e.gamepad.axes.length} axes, ${e.gamepad.buttons.length} buttons.`);
  populateAxesSelects(e.gamepad.axes.length);

  // Ensure mapping indices are valid for this gamepad
  for (const k in state.map) {
      if (state.map[k] >= e.gamepad.axes.length) state.map[k] = 0;
  }
  selects.roll.value = state.map.roll;
  selects.pitch.value = state.map.pitch;
  selects.yaw.value = state.map.yaw;
  selects.throttle.value = state.map.throttle;

  // Start poll loop if not already running
  if (!pollHandle) loopGamepad();
});

window.addEventListener("gamepaddisconnected", (e) => {
  setGamepadStatus('Gamepad disconnected.');
  gpIndex = null;
  doc('rawAxes').innerHTML = '';
});

// FIX: Synchronize checkbox state on page load and when changed
function syncCheckboxState() {
  state.invert.pitch = invertPitch.checked;
  state.invert.throttle = invertThrottle.checked;
}

// Initial sync of checkbox state
syncCheckboxState();

// Update state when checkboxes change
invertPitch.addEventListener('change', syncCheckboxState);
invertThrottle.addEventListener('change', syncCheckboxState);

// Update mapping when select dropdowns change
selects.roll.addEventListener('change', () => state.map.roll = +selects.roll.value);
selects.pitch.addEventListener('change', () => state.map.pitch = +selects.pitch.value);
selects.yaw.addEventListener('change', () => state.map.yaw = +selects.yaw.value);
selects.throttle.addEventListener('change', () => state.map.throttle = +selects.throttle.value);

// Auto-map heuristic: assign axes in order (0=roll, 1=pitch, 2=yaw, 3=throttle)
autoMapBtn.addEventListener('click', () => {
  const gps = navigator.getGamepads ? navigator.getGamepads() : [];
  const gp = gps[gpIndex] || null;
  const axes = gp ? gp.axes.length : 6;
  state.map.roll = Math.min(0, axes - 1);
  state.map.pitch = Math.min(1, axes - 1);
  state.map.yaw = Math.min(2, axes - 1);
  state.map.throttle = Math.min(3, axes - 1);
  selects.roll.value = state.map.roll;
  selects.pitch.value = state.map.pitch;
  selects.yaw.value = state.map.yaw;
  selects.throttle.value = state.map.throttle;
  setGamepadStatus('Auto-mapped axes (0→roll, 1→pitch, 2→yaw, 3→throttle).');
});

// Calibrate zero: record current axis positions as zero offsets
calibrateZeroBtn.addEventListener('click', () => {
  const gps = navigator.getGamepads ? navigator.getGamepads() : [];
  const gp = gps[gpIndex];
  if (!gp) {
      setGamepadStatus('No gamepad to calibrate.');
      return;
  }
  state.zeroOffsets.roll = gp.axes[state.map.roll] || 0;
  state.zeroOffsets.pitch = gp.axes[state.map.pitch] || 0;
  state.zeroOffsets.yaw = gp.axes[state.map.yaw] || 0;
  state.zeroOffsets.throttle = gp.axes[state.map.throttle] || 0;
  setGamepadStatus('Calibrated current positions as zero offsets.');
});

// Rescan: manually search for a connected gamepad
connectHintBtn.addEventListener('click', () => {
  const gps = navigator.getGamepads();
  let found = false;
  for (let i = 0; i < gps.length; i++) {
      if (gps[i]) {
          gpIndex = i;
          found = true;
          populateAxesSelects(gps[i].axes.length);
          break;
      }
  }
  setGamepadStatus(found ? `Found gamepad: ${gps[gpIndex].id}` : 'No gamepad found. Connect a controller and move a control.');
  // Start polling if we found a gamepad and aren't already polling
  if (found && !pollHandle) loopGamepad();
});

// Toggle polling on/off
stopPollingBtn.addEventListener('click', () => {
  pollingGamepad = !pollingGamepad;
  stopPollingBtn.textContent = pollingGamepad ? 'Stop' : 'Resume';
  if (pollingGamepad) loopGamepad();
});

// Learn buttons: detect which axis the user moves
Object.keys(learnBtns).forEach(control => {
  learnBtns[control].addEventListener('click', () => {
      listeningFor = { control, startedAt: performance.now() };
      setGamepadStatus(`Learning ${control} — move the desired control (within 3s).`);
      setTimeout(() => {
          if (listeningFor && listeningFor.control === control) {
              listeningFor = null;
              setGamepadStatus('Learn timed out. Try again.');
          }
      }, 3000);
  });
});

// Detect which axis moved the most since last frame
function detectMovedAxis(axes, lastAxesLocal) {
  let bestIdx = -1;
  let bestDelta = 0.05;  // Threshold to detect meaningful movement
  for (let i = 0; i < axes.length; i++) {
      const prev = (lastAxesLocal && lastAxesLocal[i] != null) ? lastAxesLocal[i] : 0;
      const delta = Math.abs(axes[i] - prev);
      if (delta > bestDelta) {
          bestDelta = delta;
          bestIdx = i;
      }
  }
  return bestIdx;
}

// Read a mapped axis from the gamepad, applying zero offset and clamping
function readAxisFromGamepad(gp, key) {
  const idx = state.map[key];
  let v = 0;
  if (idx != null && idx < gp.axes.length) v = gp.axes[idx];
  v = v - (state.zeroOffsets[key] || 0);
  v = fxClamp(v, -1, 1);
  return v;
}

// Main polling loop: runs every frame to read gamepad input and update UI
function loopGamepad() {
  // Stop if polling is disabled
  if (!pollingGamepad) return;

  // Find a connected gamepad
  const gps = navigator.getGamepads ? navigator.getGamepads() : [];
  let gp = null;

  if (gpIndex != null && gps[gpIndex]) {
      gp = gps[gpIndex];
  } else {
      // Try to find any connected gamepad
      for (let i = 0; i < gps.length; i++) {
          if (gps[i]) {
              gp = gps[i];
              gpIndex = i;
              break;
          }
      }
  }

  // If no gamepad found, display message and continue polling
  if (!gp) {
      setGamepadStatus('No gamepad connected. Connect controller / joystick and move controls.');
      doc('rawAxes').innerHTML = '';
      lastAxes = [];
      pollHandle = requestAnimationFrame(loopGamepad);
      return;
  }

  // Display gamepad info
  setGamepadStatus(`Gamepad: ${gp.id} — axes: ${gp.axes.length}, buttons: ${gp.buttons.length}`);

  // Show raw axis values as chips
  doc('rawAxes').innerHTML = '';
  gp.axes.forEach((v, i) => {
      const chip = document.createElement('div');
      chip.className = 'chip';
      chip.textContent = `A${i}: ${Number(v).toFixed(3)}`;
      doc('rawAxes').appendChild(chip);
  });

  // Learn: detect if user moved an axis
  if (listeningFor) {
      const idx = detectMovedAxis(gp.axes, lastAxes);
      if (idx >= 0) {
          const c = listeningFor.control;
          state.map[c] = idx;
          selects[c].value = String(idx);
          setGamepadStatus(`Assigned axis ${idx} → ${c}.`);
          listeningFor = null;
      }
  }

  // Read mapped axes with zero offset applied
  let rawRoll = readAxisFromGamepad(gp, 'roll');
  let rawPitch = readAxisFromGamepad(gp, 'pitch');
  let rawYaw = readAxisFromGamepad(gp, 'yaw');
  let rawThrottle = readAxisFromGamepad(gp, 'throttle');

  // Apply inversion settings
  if (state.invert.pitch) rawPitch = -rawPitch;

  // Convert throttle from [-1..1] to [1200..1800] PWM range
  let thr01 = (rawThrottle + 1) / 2;  // Convert to [0..1]
  if (state.invert.throttle) thr01 = 1 - thr01;
  thr01 = fxClamp(thr01, 0, 1);
  const throttleVal = Math.round(thr01 * (1800 - 1200) + 1200);

  // Convert pitch/roll/yaw to -45..45 degree range
  const pitchVal = Math.round(fxClamp(rawPitch * 45, -45, 45));
  const rollVal = Math.round(fxClamp(rawRoll * 45, -45, 45));
  const yawVal = Math.round(fxClamp(rawYaw * 45, -45, 45));

  // Update the control sliders (these are the IDs the WebSocket code reads)
  const ut = doc('userThrottleSlider');
  const up = doc('userPitchSlider');
  const ur = doc('userRollSlider');
  const uy = doc('userYawSlider');

  if (ut && +ut.value !== throttleVal) ut.value = throttleVal;
  if (up && +up.value !== pitchVal) up.value = pitchVal;
  if (ur && +ur.value !== rollVal) ur.value = rollVal;
  if (uy && +uy.value !== yawVal) uy.value = yawVal;

  // Update the display labels for the sliders
  try {
      if (typeof updateUserPitch === 'function') updateUserPitch();
      if (typeof updateUserRoll === 'function') updateUserRoll();
      if (typeof updateUserYaw === 'function') updateUserYaw();
      if (typeof updateUserThrottle === 'function') updateUserThrottle();
  } catch (e) {
      console.warn('updateUser functions not available yet', e);
  }

  // Save axes for next frame's delta detection
  lastAxes = gp.axes.slice();

  // Schedule next frame
  pollHandle = requestAnimationFrame(loopGamepad);
}

// FIX: On page load, manually check for already-connected gamepads
// This handles the case where a user has a gamepad plugged in before loading the page
window.addEventListener('load', function () {
  setTimeout(() => {
      const gps = navigator.getGamepads ? navigator.getGamepads() : [];
      for (let i = 0; i < gps.length; i++) {
          if (gps[i]) {
              gpIndex = i;
              populateAxesSelects(gps[i].axes.length);
              setGamepadStatus(`Gamepad found on startup: ${gps[i].id}`);
              loopGamepad();
              break;
          }
      }
  }, 500);  // Small delay to ensure event listeners are attached
});

// Make sure user sliders still work when manually adjusted
['userThrottleSlider', 'userPitchSlider', 'userRollSlider', 'userYawSlider'].forEach(id => {
  const el = doc(id);
  if (!el) return;
  el.addEventListener('input', () => {
      if (id === 'userThrottleSlider') updateUserThrottle();
      if (id === 'userPitchSlider') updateUserPitch();
      if (id === 'userRollSlider') updateUserRoll();
      if (id === 'userYawSlider') updateUserYaw();
  });
});