let ws = null;
let autoTelemetryEnabled = false;
let commandSendInterval = null;

function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = protocol + '//' + window.location.hostname + ':8009';
    ws = new WebSocket(wsUrl);

    ws.onopen = function() {
        console.log('WebSocket connected');
        document.getElementById('status').style.color = 'green';
        document.getElementById('status').innerText = 'Connected';

        // Start sending control commands at safe refresh rate
        if (!commandSendInterval) {
            commandSendInterval = setInterval(sendControlCommands, 25); // 40Hz - but server adds 5ms delay
        }
    };

    ws.onmessage = function(event) {
        const data = event.data;
        document.getElementById('parsedDataBox').innerText = data;
        const parsed = parseDataFurther(data);
        updateDisplayValues(parsed);
    };

    ws.onerror = function(error) {
        console.error('WebSocket error:', error);
        document.getElementById('status').style.color = 'orange';
        document.getElementById('status').innerText = 'Error';
    };

    ws.onclose = function() {
        console.log('WebSocket disconnected');
        document.getElementById('status').style.color = 'red';
        document.getElementById('status').innerText = 'Disconnected';
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

    const command = '.t' + throttle + ";" + ".p" + pitch + ";" + ".r" + roll + ";" + ".y" + yaw + ";" + ".s";
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
        setTimeout(autoTelemetryLoop, 5000); // Request telemetry every 5 seconds
    }
}

function updateDisplayValues(parsed) {
    document.getElementById('throttleSlider').value = parsed.throttle;
    document.getElementById('throttleValue').innerText = parsed.throttle.toFixed(2);
    document.getElementById('pitchSlider').value = parsed.actualPitch;
    document.getElementById('actualPitchValue').innerText = parsed.actualPitch.toFixed(2);
    document.getElementById('rollSlider').value = parsed.actualRoll;
    document.getElementById('actualRollValue').innerText = parsed.actualRoll.toFixed(2);
    document.getElementById('yawSlider').value = parsed.actualYaw;
    document.getElementById('actualYawValue').innerText = parsed.actualYaw.toFixed(2);
    document.getElementById('targetPitchSlider').value = parsed.targetPitch;
    document.getElementById('targetPitchValue').innerText = parsed.targetPitch.toFixed(2);
    document.getElementById('targetRollSlider').value = parsed.targetRoll;
    document.getElementById('targetRollValue').innerText = parsed.targetRoll.toFixed(2);
    document.getElementById('targetYawSlider').value = parsed.targetYaw;
    document.getElementById('targetYawValue').innerText = parsed.targetYaw.toFixed(2);
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
        pidOutputYaw: parseFloat(dataArray[9]) || 0
    };
}

// Initialize WebSocket connection on page load
window.addEventListener('load', function() {
    setTimeout(connectWebSocket, 1000);
});

/* --------------------------------------------------------
   Second: Gamepad mapping code (mapping + learn + UI)
   This block complements the UI and updates the user* sliders
   — it does not modify your WebSocket logic above.
   -------------------------------------------------------- */

const doc = id => document.getElementById(id);

let gpIndex = null;
let pollingGamepad = true;
let pollHandle = null;
let lastAxes = [];
let listeningFor = null; // { control, startedAt }

const state = {
  map: { roll: 0, pitch: 1, yaw: 2, throttle: 3 },
  invert: { pitch: false, throttle: false },
  zeroOffsets: { roll:0, pitch:0, yaw:0, throttle:0 }
};

// elements for mapping UI
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

function fxClamp(v,a,b){ return Math.max(a, Math.min(b, v)); }
function setStatus(text){ doc('gpStatus').textContent = text; }

function populateAxesSelects(count){
  [selects.roll, selects.pitch, selects.yaw, selects.throttle].forEach(s => {
    const prev = s.value;
    s.innerHTML = '';
    for(let i=0;i<count;i++){
      const opt = document.createElement('option');
      opt.value = String(i);
      opt.textContent = `Axis ${i}`;
      s.appendChild(opt);
    }
    if(prev !== '' && +prev < count) s.value = prev;
  });
}

// seed selects with default 6
populateAxesSelects(6);

// gamepad connect/disconnect
window.addEventListener("gamepadconnected", (e) => {
  gpIndex = e.gamepad.index;
  setStatus(`Gamepad connected: ${e.gamepad.id} — ${e.gamepad.axes.length} axes, ${e.gamepad.buttons.length} buttons.`);
  populateAxesSelects(e.gamepad.axes.length);
  // ensure mapping indices valid
  for (const k in state.map) {
    if (state.map[k] >= e.gamepad.axes.length) state.map[k] = 0;
  }
  selects.roll.value = state.map.roll;
  selects.pitch.value = state.map.pitch;
  selects.yaw.value = state.map.yaw;
  selects.throttle.value = state.map.throttle;
  // start poll loop
  if (!pollHandle) loopGamepad();
});
window.addEventListener("gamepaddisconnected", (e) => {
  setStatus('Gamepad disconnected.');
  gpIndex = null;
  doc('rawAxes').innerHTML = '';
});

// update mapping from selects
selects.roll.addEventListener('change', () => state.map.roll = +selects.roll.value);
selects.pitch.addEventListener('change', () => state.map.pitch = +selects.pitch.value);
selects.yaw.addEventListener('change', () => state.map.yaw = +selects.yaw.value);
selects.throttle.addEventListener('change', () => state.map.throttle = +selects.throttle.value);
invertPitch.addEventListener('change', () => state.invert.pitch = invertPitch.checked);
invertThrottle.addEventListener('change', () => state.invert.throttle = invertThrottle.checked);

// auto-map heuristic
autoMapBtn.addEventListener('click', () => {
  const gps = navigator.getGamepads ? navigator.getGamepads() : [];
  const gp = gps[gpIndex] || null;
  const axes = gp ? gp.axes.length : 6;
  state.map.roll = Math.min(0, axes-1);
  state.map.pitch = Math.min(1, axes-1);
  state.map.yaw = Math.min(2, axes-1);
  state.map.throttle = Math.min(3, axes-1);
  selects.roll.value = state.map.roll;
  selects.pitch.value = state.map.pitch;
  selects.yaw.value = state.map.yaw;
  selects.throttle.value = state.map.throttle;
  setStatus('Auto-mapped axes (0→roll,1→pitch,2→yaw,3→throttle).');
});

// calibrate zero
calibrateZeroBtn.addEventListener('click', () => {
  const gps = navigator.getGamepads ? navigator.getGamepads() : [];
  const gp = gps[gpIndex];
  if (!gp){ setStatus('No gamepad to calibrate.'); return; }
  state.zeroOffsets.roll = gp.axes[state.map.roll] || 0;
  state.zeroOffsets.pitch = gp.axes[state.map.pitch] || 0;
  state.zeroOffsets.yaw = gp.axes[state.map.yaw] || 0;
  state.zeroOffsets.throttle = gp.axes[state.map.throttle] || 0;
  setStatus('Calibrated current positions as zero offsets.');
});

// connect hint (rescan)
connectHintBtn.addEventListener('click', () => {
  const gps = navigator.getGamepads();
  let found = false;
  for (let i=0;i<gps.length;i++){
    if (gps[i]) { gpIndex = i; found = true; populateAxesSelects(gps[i].axes.length); break; }
  }
  setStatus(found ? `Found gamepad: ${gps[gpIndex].id}` : 'No gamepad found. Connect a controller and move a control.');
});

// stop / resume polling
stopPollingBtn.addEventListener('click', () => {
  pollingGamepad = !pollingGamepad;
  stopPollingBtn.textContent = pollingGamepad ? 'Stop' : 'Resume';
  if (pollingGamepad) loopGamepad();
});

// Learn buttons
Object.keys(learnBtns).forEach(control => {
  learnBtns[control].addEventListener('click', () => {
    listeningFor = { control, startedAt: performance.now() };
    setStatus(`Learning ${control} — move the desired control (within 3s).`);
    setTimeout(() => {
      if (listeningFor && listeningFor.control === control) {
        listeningFor = null;
        setStatus('Learn timed out. Try again.');
      }
    }, 3000);
  });
});

function detectMovedAxis(axes, lastAxesLocal){
  let bestIdx = -1;
  let bestDelta = 0.05;
  for (let i=0;i<axes.length;i++){
    const prev = (lastAxesLocal && lastAxesLocal[i] != null) ? lastAxesLocal[i] : 0;
    const delta = Math.abs(axes[i] - prev);
    if (delta > bestDelta){
      bestDelta = delta;
      bestIdx = i;
    }
  }
  return bestIdx;
}

function readAxisFromGamepad(gp, key){
  const idx = state.map[key];
  let v = 0;
  if (idx != null && idx < gp.axes.length) v = gp.axes[idx];
  v = v - (state.zeroOffsets[key] || 0);
  v = fxClamp(v, -1, 1);
  return v;
}

// Main polling loop for mapping
function loopGamepad(){
  if (!pollingGamepad) return;
  const gps = navigator.getGamepads ? navigator.getGamepads() : [];
  let gp = null;
  if (gpIndex != null && gps[gpIndex]) gp = gps[gpIndex];
  else {
    for (let i=0;i<gps.length;i++) if (gps[i]) { gp = gps[i]; gpIndex = i; break; }
  }

  if (!gp){
    doc('gpStatus').textContent = 'No gamepad connected. Connect controller / joystick and move controls.';
    doc('rawAxes').innerHTML = '';
    lastAxes = [];
    pollHandle = requestAnimationFrame(loopGamepad);
    return;
  }

  // show gamepad id
  doc('gpStatus').textContent = `Gamepad: ${gp.id} — axes: ${gp.axes.length}, buttons: ${gp.buttons.length}`;

  // raw axes
  doc('rawAxes').innerHTML = '';
  gp.axes.forEach((v,i) => {
    const chip = document.createElement('div');
    chip.className = 'chip';
    chip.textContent = `A${i}: ${Number(v).toFixed(3)}`;
    doc('rawAxes').appendChild(chip);
  });

  // Learn detection
  if (listeningFor){
    const idx = detectMovedAxis(gp.axes, lastAxes);
    if (idx >= 0){
      const c = listeningFor.control;
      state.map[c] = idx;
      selects[c].value = String(idx);
      setStatus(`Assigned axis ${idx} → ${c}.`);
      listeningFor = null;
    }
  }

  // read mapped axes
  let rawRoll = readAxisFromGamepad(gp,'roll');
  let rawPitch = readAxisFromGamepad(gp,'pitch');
  let rawYaw = readAxisFromGamepad(gp,'yaw');
  let rawThrottle = readAxisFromGamepad(gp,'throttle');

  if (state.invert.pitch) rawPitch = -rawPitch;

  // throttle mapping: [-1..1] -> [1200..1800]
  let thr01 = (rawThrottle + 1) / 2;
  if (state.invert.throttle) thr01 = 1 - thr01;
  thr01 = fxClamp(thr01, 0, 1);
  const throttleVal = Math.round(thr01 * (1800 - 1200) + 1200);

  // convert pitch/roll/yaw to -45..45
  const pitchVal = Math.round(fxClamp(rawPitch * 45, -45, 45));
  const rollVal = Math.round(fxClamp(rawRoll * 45, -45, 45));
  const yawVal = Math.round(fxClamp(rawYaw * 45, -45, 45));

  // update the send sliders (these are the exact DOM IDs your WS code reads)
  const ut = doc('userThrottleSlider'), up = doc('userPitchSlider'), ur = doc('userRollSlider'), uy = doc('userYawSlider');
  if (ut && +ut.value !== throttleVal) ut.value = throttleVal;
  if (up && +up.value !== pitchVal) up.value = pitchVal;
  if (ur && +ur.value !== rollVal) ur.value = rollVal;
  if (uy && +uy.value !== yawVal) uy.value = yawVal;

  // trigger the existing updateUser... functions so UI numeric labels update exactly as before
  try {
    if (typeof updateUserPitch === 'function') updateUserPitch();
    if (typeof updateUserRoll === 'function') updateUserRoll();
    if (typeof updateUserYaw === 'function') updateUserYaw();
    if (typeof updateUserThrottle === 'function') updateUserThrottle();
  } catch (e) {
    console.warn('updateUser functions not available yet', e);
  }

  lastAxes = gp.axes.slice();
  pollHandle = requestAnimationFrame(loopGamepad);
}

/* --------------------------------------------------------
   End gamepad mapping block
   -------------------------------------------------------- */

/* Make sure user sliders call the same update functions when manually adjusted */
['userThrottleSlider','userPitchSlider','userRollSlider','userYawSlider'].forEach(id => {
  const el = doc(id);
  if (!el) return;
  el.addEventListener('input', () => {
    if (id === 'userThrottleSlider') updateUserThrottle();
    if (id === 'userPitchSlider') updateUserPitch();
    if (id === 'userRollSlider') updateUserRoll();
    if (id === 'userYawSlider') updateUserYaw();
  });
});