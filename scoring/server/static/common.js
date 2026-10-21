// 점수판/심판 페이지 공통: 서버 상태를 WebSocket으로 받고, 끊기면 자동 재연결
function connectState(onState, onConnection) {
  function open() {
    var ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws');
    ws.onopen = function () { if (onConnection) onConnection(true); };
    ws.onmessage = function (e) {
      var state = JSON.parse(e.data);
      state.receivedAt = performance.now();
      onState(state);
    };
    ws.onclose = function () {
      if (onConnection) onConnection(false);
      setTimeout(open, 1000);
    };
    ws.onerror = function () { ws.close(); };
  }
  open();
}

// 서버가 보낸 남은 시간에서 받은 뒤 흐른 시간을 빼서 화면 시계를 부드럽게 갱신
function remainingMs(state) {
  if (state.status !== 'running') return state.remaining_ms;
  return Math.max(0, state.remaining_ms - (performance.now() - state.receivedAt));
}

function formatClock(ms) {
  var total = Math.ceil(ms / 1000);
  return Math.floor(total / 60) + ':' + ('0' + (total % 60)).slice(-2);
}

function api(body) {
  return fetch('/api/admin', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body)
  }).then(function (r) { return r.json(); });
}
