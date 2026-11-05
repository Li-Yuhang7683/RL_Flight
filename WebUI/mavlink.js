import {nearestSample, numericRange, visibleSegments, csvCell} from './mavlink_plot.mjs';

const $ = (selector, root = document) => root.querySelector(selector);
const colors = ['#138d7d', '#4d7bd2', '#da9b3b', '#ce6475', '#8869bb', '#51a8b8', '#9b9555', '#567c89'];
const state = {
  fields: new Map(), series: new Map(), history: [], charts: [], active: '',
  epoch: '', cursor: 0, offset: 0, paused: false, end: null, window: 30,
  filter: 'all', search: '', source: '', connection: {}, stats: {},
  serverTime: 0, pollError: '', nextId: 1,
};
const chartElements = new Map();
let lastSidebar = 0, lastDraw = 0, fieldSignature = '', sourcesSignature = '';

function notice(message = '') {
  $('#notice').textContent = message;
  $('#notice').hidden = !message;
}
async function api(path, body) {
  const response = await fetch(path, body === undefined ? {} : {
    method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body),
  });
  const payload = await response.json();
  if (!response.ok) throw new Error(payload.error || `HTTP ${response.status}`);
  return payload;
}
function format(value) {
  if (!Number.isFinite(value)) return '—';
  if (Math.abs(value) >= 10000 || (value !== 0 && Math.abs(value) < .001)) return value.toExponential(2);
  return value.toFixed(3);
}
function now() { return Date.now() / 1000 + state.offset; }
function viewEnd() { return state.end ?? now(); }
function isStale(field) {
  return !Number.isFinite(field?.value) || now() - field.time > Math.max(1, (field.period || .1) * 3);
}
function saveLayout() {
  try {
    localStorage.setItem('rlflight-mavlink-layout-v1', JSON.stringify({
      charts: state.charts, window: state.window, preset: $('#preset').value,
    }));
  } catch { /* Storage may be unavailable in private browser profiles. */ }
}
function makeChart(title, wanted = null) {
  const chart = {id: `chart-${state.nextId++}`, title, traces: [], min: null, max: null, wanted};
  state.charts.push(chart);
  return chart;
}
function setPreset(preset) {
  state.charts = [];
  if (preset === 'compare') {
    for (const aircraft of ['A', 'B']) for (const sensor of ['ACC', 'GYR']) {
      makeChart(`${aircraft === 'A' ? '主机 A' : '从机 B'} · ${sensor === 'ACC' ? '加速度' : '角速度'} R / C`,
        ['R', 'C'].flatMap(frame => ['X', 'Y', 'Z'].map(axis => `${aircraft}_${sensor}_${frame}_${axis}`)));
    }
  } else if (preset === 'cekf') {
    makeChart('CEKF · 姿态 / yaw 参考', ['C_ROLL', 'C_PITCH', 'C_YAW', 'CE_YAWREF']);
    makeChart('CEKF · 虚拟比力', ['C_ACC_X', 'C_ACC_Y', 'C_ACC_Z']);
    makeChart('CEKF · 虚拟角速度', ['C_GYR_X', 'C_GYR_Y', 'C_GYR_Z']);
    makeChart('CEKF · 角加速度', ['C_ALP_X', 'C_ALP_Y', 'C_ALP_Z']);
    makeChart('CEKF · IMU 权重', ['CE_W_A', 'CE_W_B']);
    makeChart('CEKF · 连接与有效状态', ['CE_LINK', 'CE_VALID', 'CE_INIT']);
  } else if (preset === 'cekf-status') {
    makeChart('CEKF · 运行与初始化', ['CE_RUN', 'CE_EN', 'CE_INIT', 'CE_VALID']);
    makeChart('CEKF · 模式与连接', ['CE_STATE', 'CE_LINK']);
    makeChart('CEKF · IMU 权重', ['CE_W_A', 'CE_W_B']);
    makeChart('CEKF · 重置计数', ['CE_RESET']);
    makeChart('CEKF · 输入年龄', ['CE_AGE_A', 'CE_AGE_B']);
    makeChart('CEKF · 健康标志', ['CE_FLAGS']);
    makeChart('CEKF · 双机融合样本计数', ['CE_FRAMES']);
  } else {
    const frame = preset === 'raw' ? 'R' : 'C';
    for (const sensor of ['ACC', 'GYR']) for (const axis of ['X', 'Y', 'Z']) {
      makeChart(`${sensor === 'ACC' ? '加速度' : '角速度'} ${axis} · A / B ${frame}`,
        ['A', 'B'].map(aircraft => `${aircraft}_${sensor}_${frame}_${axis}`));
    }
  }
  state.active = state.charts[0].id;
  fillPreset();
  renderCharts();
  renderFields();
  saveLayout();
}
function fillPreset() {
  const fields = [...state.fields.values()];
  let changed = false;
  for (const chart of state.charts) {
    if (!chart.wanted) continue;
    for (const name of chart.wanted) {
      if (chart.traces.some(trace => state.fields.get(trace.id)?.name === name)) continue;
      const field = fields.find(item => item.name === name);
      if (field) { chart.traces.push({id: field.id, side: 'left'}); changed = true; }
    }
  }
  if (changed) { renderCharts(); renderFields(); }
}
function setActive(id) {
  state.active = id;
  for (const [key, node] of chartElements) node.classList.toggle('active', key === id);
  $('#activeChartLabel').textContent = state.charts.find(chart => chart.id === id)?.title || '当前图表';
  renderFields();
}
function removeTrace(chart, fieldId) {
  chart.wanted = null;
  chart.traces = chart.traces.filter(trace => trace.id !== fieldId);
  renderLegend(chart);
  renderFields();
  saveLayout();
}
function toggleTrace(id, checked) {
  let chart = state.charts.find(item => item.id === state.active);
  if (!chart) { chart = makeChart('自定义图表'); state.active = chart.id; renderCharts(); }
  chart.wanted = null;
  if (checked && !chart.traces.some(trace => trace.id === id)) {
    const unit = state.fields.get(id)?.unit;
    const leftUnits = chart.traces.filter(t => t.side === 'left').map(t => state.fields.get(t.id)?.unit);
    chart.traces.push({id, side: unit && leftUnits.some(u => u && u !== unit) ? 'right' : 'left'});
  } else if (!checked) chart.traces = chart.traces.filter(trace => trace.id !== id);
  renderLegend(chart);
  saveLayout();
}
function renderLegend(chart) {
  const node = chartElements.get(chart.id);
  if (!node) return;
  const legend = $('.chart-legend', node);
  legend.replaceChildren();
  chart.traces.forEach((trace, index) => {
    const field = state.fields.get(trace.id);
    const item = document.createElement('div');
    item.className = 'legend-item';
    const line = document.createElement('i');
    line.style.background = colors[index % colors.length];
    const name = document.createElement('span');
    name.className = 'legend-name';
    name.textContent = field?.name || trace.id.split('/').at(-1);
    name.title = `${field?.label || name.textContent}\n${trace.id}`;
    const value = document.createElement('span');
    value.className = 'legend-value';
    value.dataset.field = trace.id;
    const side = document.createElement('select');
    side.setAttribute('aria-label', `${name.textContent} 纵轴`);
    for (const [key, label] of [['left', '左'], ['right', '右']]) {
      const option = new Option(label, key); side.add(option);
    }
    side.value = trace.side;
    side.onchange = () => { trace.side = side.value; saveLayout(); };
    const remove = document.createElement('button');
    remove.className = 'legend-remove'; remove.textContent = '×';
    remove.setAttribute('aria-label', `移除 ${name.textContent}`);
    remove.onclick = () => removeTrace(chart, trace.id);
    item.append(line, name, value, side, remove);
    legend.append(item);
  });
  $('.chart-empty', node).hidden = chart.traces.length > 0;
  $('.chart-traces', node).textContent = `${chart.traces.length} 条曲线`;
}
function renderCharts() {
  const grid = $('#chartGrid');
  for (const [id, node] of chartElements) {
    if (!state.charts.some(chart => chart.id === id)) { node.remove(); chartElements.delete(id); }
  }
  state.charts.forEach((chart, index) => {
    let node = chartElements.get(chart.id);
    if (!node) {
      node = $('#chartTemplate').content.firstElementChild.cloneNode(true);
      chartElements.set(chart.id, node);
      grid.append(node);
      const title = $('.chart-title', node);
      title.value = chart.title;
      title.onchange = () => { chart.title = title.value.trim() || '自定义图表'; title.value = chart.title; setActive(chart.id); saveLayout(); };
      node.addEventListener('pointerdown', () => { if (state.active !== chart.id) setActive(chart.id); });
      node.addEventListener('focusin', () => { if (state.active !== chart.id) setActive(chart.id); });
      $('.chart-remove', node).onclick = () => {
        state.charts = state.charts.filter(item => item.id !== chart.id);
        if (state.active === chart.id) state.active = state.charts[0]?.id || '';
        renderCharts(); renderFields(); saveLayout();
      };
      $('.chart-scale', node).onclick = () => {
        const panel = $('.scale-panel', node); panel.hidden = !panel.hidden;
        $('.y-min', node).value = chart.min ?? ''; $('.y-max', node).value = chart.max ?? '';
      };
      $('.scale-apply', node).onclick = () => {
        const rawMin = $('.y-min', node).value, rawMax = $('.y-max', node).value;
        const min = rawMin === '' ? null : Number(rawMin), max = rawMax === '' ? null : Number(rawMax);
        if ((min !== null && !Number.isFinite(min)) || (max !== null && !Number.isFinite(max)) ||
          ((min === null) !== (max === null)) || (min !== null && min >= max)) {
          notice('纵轴范围请同时填写最小和最大值，且最大值应大于最小值。'); return;
        }
        chart.min = min; chart.max = max; $('.scale-panel', node).hidden = true; notice(); saveLayout();
      };
      $('.scale-reset', node).onclick = () => { chart.min = chart.max = null; $('.scale-panel', node).hidden = true; saveLayout(); };
      bindCanvas(chart, node);
    }
    $('.chart-index', node).textContent = String(index + 1).padStart(2, '0');
    node.classList.toggle('active', chart.id === state.active);
    renderLegend(chart);
  });
  $('#activeChartLabel').textContent = state.charts.find(chart => chart.id === state.active)?.title || '当前图表';
}
function renderFields() {
  const root = $('#signalList');
  const active = state.charts.find(chart => chart.id === state.active);
  const selected = new Set(active?.traces.map(trace => trace.id) || []);
  const search = state.search.toLowerCase();
  const fields = [...state.fields.values()].filter(field => {
    if (state.filter === 'imu' && !/^[AB]_(ACC|GYR)_[RC]_[XYZ]$/.test(field.name)) return false;
    if (state.filter === 'cekf' && !/^(C_|CE_)/.test(field.name)) return false;
    if (['A', 'B'].includes(state.filter) && !field.name.startsWith(`${state.filter}_`)) return false;
    if (state.source && `${field.system}/${field.component}` !== state.source) return false;
    return `${field.name} ${field.label} ${field.unit} ${field.id}`.toLowerCase().includes(search);
  });
  $('#signalCount').textContent = fields.length;
  if (!state.fields.size) return;
  root.replaceChildren();
  const groups = new Map();
  fields.sort((a, b) => a.group.localeCompare(b.group) || a.name.localeCompare(b.name));
  for (const field of fields) {
    const groupKey = `${field.group} · ${field.system}/${field.component} · ${field.message}`;
    let group = groups.get(groupKey);
    if (!group) {
      group = document.createElement('details'); group.className = 'signal-group'; group.open = true;
      const summary = document.createElement('summary'); summary.textContent = groupKey;
      group.append(summary); groups.set(groupKey, group); root.append(group);
    }
    const label = document.createElement('label'); label.className = 'signal-row';
    const checkbox = document.createElement('input'); checkbox.type = 'checkbox'; checkbox.checked = selected.has(field.id);
    checkbox.setAttribute('aria-label', field.name); checkbox.onchange = () => toggleTrace(field.id, checkbox.checked);
    const text = document.createElement('span');
    const name = document.createElement('span'); name.className = 'signal-name'; name.textContent = field.name;
    const desc = document.createElement('small'); desc.className = 'signal-desc'; desc.textContent = field.label;
    text.append(name, desc);
    const value = document.createElement('span'); value.className = 'signal-value'; value.dataset.field = field.id;
    label.append(checkbox, text, value); group.append(label);
  }
  if (!fields.length) { const empty = document.createElement('p'); empty.className = 'empty-signals'; empty.textContent = '没有匹配的字段'; root.append(empty); }
  updateValues();
}
function updateValues() {
  for (const node of document.querySelectorAll('.signal-value')) {
    const field = state.fields.get(node.dataset.field);
    if (!field) continue;
    node.classList.toggle('stale', isStale(field));
    node.replaceChildren(document.createTextNode(format(field.value)));
    const unit = document.createElement('small'); unit.textContent = isStale(field) ? (field.value == null ? '无效' : '过期') : field.unit;
    node.append(unit);
  }
}
function updateConnection(connection, stats) {
  state.connection = connection; state.stats = stats;
  const labels = {disconnected: '未连接', connecting: '连接中', connected: '已连接', error: '连接错误'};
  const badge = $('#connectionBadge'); badge.className = `status ${connection.state}`;
  badge.lastChild.textContent = labels[connection.state] || '未连接';
  const config = connection.config;
  const configKey = JSON.stringify(config);
  if (config && state.configKey !== configKey) {
    state.configKey = configKey;
    $('#transport').value = config.transport;
    transportChanged();
    $('#endpoint').value = config.endpoint;
    $('#baud').value = String(config.baud);
    $('#heartbeat').checked = config.heartbeat;
  }
  $('#sourceMetric').textContent = config ? (config.transport === 'demo' ? '演示数据 · 20 Hz' : `${config.transport.toUpperCase()} · ${config.endpoint}`) : '等待连接';
  $('#rateMetric').textContent = stats.messages_per_second || 0;
  $('#fieldsMetric').textContent = state.fields.size;
  $('#packetMetric').textContent = `${stats.messages || 0} 条消息 · ${stats.bad_packets || 0} 错包`;
  $('#streamStatus').textContent = connection.stream_request || '';
  const target = stats.targets?.[0];
  if (target && !$('#targetSystem').dataset.edited) {
    $('#targetSystem').value = target[0]; $('#targetComponent').value = target[1];
  }
  if (connection.error) notice(connection.error);
  const age = stats.last_packet == null ? null : Math.max(0, now() - stats.last_packet);
  $('#ageMetric').textContent = age == null ? '等待数据' : age < 1 ? `${Math.round(age * 1000)} ms 前` : `${age.toFixed(1)} s 前`;
  $('#bufferStatus').textContent = `缓存 ${state.history.length.toLocaleString()} 个采样点 · 本地接收时间${state.connection.config?.transport === 'demo' ? ' · 演示数据' : ''}`;
}
async function poll() {
  let delay = 100;
  try {
    const payload = await api(`/api/samples?cursor=${state.cursor}&epoch=${state.epoch}`);
    state.offset = payload.server_time - Date.now() / 1000;
    if (state.epoch !== payload.epoch) {
      state.epoch = payload.epoch; state.cursor = 0; state.history = [];
      state.fields.clear(); state.series.clear(); fieldSignature = ''; sourcesSignature = '';
      state.paused = false; state.end = null; updatePause();
    }
    for (const field of payload.fields) state.fields.set(field.id, field);
    for (const sample of payload.samples) {
      if (sample[0] <= state.cursor) continue;
      if (!state.series.has(sample[2])) state.series.set(sample[2], []);
      state.series.get(sample[2]).push(sample); state.history.push(sample);
    }
    state.cursor = payload.cursor;
    const cutoff = payload.server_time - 300;
    let remove = Math.max(0, state.history.length - 120000);
    while (remove < state.history.length && state.history[remove][1] < cutoff) remove++;
    if (remove) state.history.splice(0, remove);
    const oldest = state.history[0]?.[0] ?? Infinity;
    for (const samples of state.series.values()) {
      let count = 0;
      while (count < samples.length && samples[count][0] < oldest) count++;
      if (count) samples.splice(0, count);
    }
    if (payload.truncated) notice('接收缓存已满，较早数据已被淘汰；曲线继续显示保留的数据。');
    const signature = [...state.fields.keys()].join('|');
    if (signature !== fieldSignature) {
      fieldSignature = signature; fillPreset(); renderLegendAll(); renderFields();
    }
    const sources = [...new Set([...state.fields.values()].map(field => `${field.system}/${field.component}`))].sort();
    if (sources.join(',') !== sourcesSignature) {
      sourcesSignature = sources.join(','); const select = $('#sourceFilter');
      select.replaceChildren(new Option('所有系统 / 组件', ''));
      for (const source of sources) select.add(new Option(source, source));
      if (!sources.includes(state.source)) state.source = '';
      select.value = state.source;
    }
    updateConnection(payload.connection, payload.stats);
    if (state.pollError) { state.pollError = ''; if (!payload.connection.error) notice(); }
    delay = payload.has_more ? 0 : 100;
  } catch (error) {
    state.pollError = error.message;
    notice(`本地服务连接失败：${error.message}`);
    $('#connectionBadge').className = 'status error';
    $('#connectionBadge').lastChild.textContent = '服务离线';
    delay = 1000;
  }
  setTimeout(poll, delay);
}
function renderLegendAll() { for (const chart of state.charts) renderLegend(chart); }
function updatePause() {
  $('#pauseButton').textContent = state.paused ? '▶ 继续实时' : 'Ⅱ 暂停查看';
  $('#liveState').classList.toggle('paused', state.paused);
  $('#liveState span').textContent = state.paused ? '历史查看' : '实时跟随';
}
function resume() { state.paused = false; state.end = null; updatePause(); }
function bindCanvas(chart, node) {
  const canvas = $('canvas', node);
  let drag = null;
  canvas.addEventListener('wheel', event => {
    event.preventDefault();
    const rect = canvas.getBoundingClientRect();
    const fraction = Math.max(0, Math.min(1, (event.clientX - rect.left - 50) / (rect.width - 90)));
    const oldWindow = state.window, end = viewEnd();
    const point = end - oldWindow * (1 - fraction);
    state.window = Math.max(1, Math.min(300, state.window * Math.exp(event.deltaY * .002)));
    state.end = Math.min(now(), point + state.window * (1 - fraction));
    state.paused = true; updatePause();
  }, {passive: false});
  canvas.addEventListener('pointerdown', event => {
    drag = {x: event.clientX, end: viewEnd()};
    canvas.setPointerCapture(event.pointerId);
  });
  canvas.addEventListener('pointermove', event => {
    const rect = canvas.getBoundingClientRect();
    if (drag && Math.abs(event.clientX - drag.x) > 2) {
      const oldest = state.history[0]?.[1] ?? now() - 300;
      state.end = Math.max(oldest, Math.min(now(), drag.end - (event.clientX - drag.x) / Math.max(rect.width - 90, 1) * state.window));
      state.paused = true; updatePause();
    }
    chart.hover = Math.max(0, Math.min(1, (event.clientX - rect.left - 50) / Math.max(rect.width - 90, 1)));
  });
  const stopDrag = () => { drag = null; };
  canvas.addEventListener('pointerup', stopDrag); canvas.addEventListener('pointercancel', stopDrag);
  canvas.addEventListener('pointerleave', () => { if (!drag) chart.hover = null; });
  canvas.addEventListener('dblclick', () => { chart.hover = null; resume(); });
}
function drawChart(chart) {
  const node = chartElements.get(chart.id);
  if (!node) return;
  const canvas = $('canvas', node), rect = canvas.getBoundingClientRect();
  const width = rect.width, height = rect.height, ratio = window.devicePixelRatio || 1;
  if (!width || !height) return;
  const pixelWidth = Math.round(width * ratio), pixelHeight = Math.round(height * ratio);
  if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) { canvas.width = pixelWidth; canvas.height = pixelHeight; }
  const ctx = canvas.getContext('2d'); ctx.setTransform(ratio, 0, 0, ratio, 0, 0); ctx.clearRect(0, 0, width, height);
  const box = {left: 50, top: 17, width: Math.max(1, width - 90), height: height - 48};
  const end = viewEnd(), start = end - state.window;
  const traces = chart.traces.map((trace, index) => {
    const field = state.fields.get(trace.id);
    return {...trace, field, color: colors[index % colors.length], segments: visibleSegments(
      state.series.get(trace.id) || [], start, end, box.width, Math.max(.5, (field?.period || .1) * 3))};
  });
  const ranges = {};
  for (const side of ['left', 'right']) {
    const values = traces.filter(trace => trace.side === side).flatMap(trace => trace.segments.flatMap(segment => segment.map(sample => sample[3])));
    ranges[side] = numericRange(values, chart.min, chart.max);
  }
  const x = timestamp => box.left + (timestamp - start) / state.window * box.width;
  const y = (value, side) => {
    const [min, max] = ranges[side]; return box.top + box.height - (value - min) / (max - min) * box.height;
  };
  ctx.font = '9px Consolas, monospace'; ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const lineY = box.top + i / 4 * box.height;
    ctx.strokeStyle = '#ecf0ef'; ctx.beginPath(); ctx.moveTo(box.left, lineY); ctx.lineTo(box.left + box.width, lineY); ctx.stroke();
    for (const side of ['left', 'right']) {
      if (side === 'right' && !traces.some(trace => trace.side === side)) continue;
      const [min, max] = ranges[side], value = max - i / 4 * (max - min);
      ctx.fillStyle = '#8f9c9d'; ctx.textAlign = side === 'left' ? 'right' : 'left';
      const label = Math.abs(value) >= 10000 || (Math.abs(value) > 0 && Math.abs(value) < .001) ? value.toExponential(0) : value.toFixed(Math.abs(max - min) > 5 ? 1 : 2);
      ctx.fillText(label, side === 'left' ? box.left - 9 : box.left + box.width + 7, lineY + 3);
    }
  }
  for (let i = 0; i <= 4; i++) {
    const lineX = box.left + i / 4 * box.width;
    ctx.strokeStyle = '#f1f4f3'; ctx.beginPath(); ctx.moveTo(lineX, box.top); ctx.lineTo(lineX, box.top + box.height); ctx.stroke();
    ctx.textAlign = i === 0 ? 'left' : i === 4 ? 'right' : 'center'; ctx.fillStyle = '#95a3a3';
    const date = new Date((start + state.window * i / 4) * 1000);
    ctx.fillText(date.toLocaleTimeString('zh-CN', {hour12: false}), lineX, height - 12);
  }
  ctx.save(); ctx.beginPath(); ctx.rect(box.left - 1, box.top - 1, box.width + 2, box.height + 2); ctx.clip();
  for (const trace of traces) {
    ctx.strokeStyle = trace.color; ctx.fillStyle = trace.color; ctx.lineWidth = 1.5;
    // C-frame traces are dashed; raw-frame traces are solid.
    ctx.setLineDash(trace.field?.name.includes('_C_') ? [5, 3] : []);
    for (const segment of trace.segments) {
      ctx.beginPath();
      segment.forEach((sample, index) => { const px = x(sample[1]), py = y(sample[3], trace.side); if (index) ctx.lineTo(px, py); else ctx.moveTo(px, py); });
      ctx.stroke();
      if (segment.length === 1) { ctx.beginPath(); ctx.arc(x(segment[0][1]), y(segment[0][3], trace.side), 2, 0, Math.PI * 2); ctx.fill(); }
    }
  }
  ctx.setLineDash([]);
  const hoverTime = chart.hover == null ? null : start + chart.hover * state.window;
  if (hoverTime !== null) {
    ctx.strokeStyle = '#869d98'; ctx.setLineDash([3, 3]); ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(x(hoverTime), box.top); ctx.lineTo(x(hoverTime), box.top + box.height); ctx.stroke();
  }
  ctx.restore();
  const tooltip = $('.plot-tooltip', node);
  tooltip.hidden = hoverTime === null || !traces.length;
  const tooltipRows = [];
  if (hoverTime !== null) tooltipRows.push(new Date(hoverTime * 1000).toLocaleTimeString('zh-CN', {hour12: false}));
  for (const trace of traces) {
    const sample = nearestSample(state.series.get(trace.id) || [], hoverTime ?? end);
    const valid = sample && sample[1] >= start && Math.abs(sample[1] - (hoverTime ?? end)) <= Math.max(1, (trace.field?.period || .1) * 3);
    const text = valid ? format(sample[3]) : '—';
    for (const value of $('.chart-legend', node).querySelectorAll('.legend-value')) {
      if (value.dataset.field === trace.id) value.textContent = text;
    }
    if (hoverTime !== null) tooltipRows.push(`${trace.field?.name || trace.id.split('/').at(-1)}  ${text} ${trace.field?.unit || ''}`);
  }
  tooltip.textContent = tooltipRows.join('\n');
  const units = side => [...new Set(traces.filter(trace => trace.side === side).map(trace => trace.field?.unit || '无单位'))].join(', ');
  $('.chart-units', node).textContent = `左轴 ${units('left') || '自动'}${traces.some(trace => trace.side === 'right') ? ` · 右轴 ${units('right')}` : ''} · R 实线 / C 虚线`;
}
function animate(timestamp) {
  if (timestamp - lastDraw >= 50) {
    for (const chart of state.charts) drawChart(chart);
    lastDraw = timestamp;
  }
  if (timestamp - lastSidebar >= 500) { updateValues(); lastSidebar = timestamp; }
  requestAnimationFrame(animate);
}
function exportCsv() {
  const selected = new Set(state.charts.flatMap(chart => chart.traces.map(trace => trace.id)));
  if (!selected.size) { notice('请先在图表中选择要导出的字段。'); return; }
  const end = viewEnd(), start = end - state.window;
  const rows = [['receive_time_utc', 'receive_unix_s', 'system', 'component', 'message', 'name', 'unit', 'value']];
  for (const sample of state.history) {
    if (!selected.has(sample[2]) || sample[1] < start || sample[1] > end) continue;
    const field = state.fields.get(sample[2]);
    if (!field) continue;
    rows.push([new Date(sample[1] * 1000).toISOString(), sample[1], field.system, field.component, field.message, field.name, field.unit, sample[3] == null ? 'NaN' : sample[3]]);
  }
  if (rows.length === 1) { notice('当前可见时间窗内没有可导出的采样点。'); return; }
  const blob = new Blob(['\uFEFF', rows.map(row => row.map(csvCell).join(',')).join('\r\n')], {type: 'text/csv;charset=utf-8'});
  const url = URL.createObjectURL(blob), link = document.createElement('a'); link.href = url;
  link.download = `mavlink-${new Date().toISOString().replaceAll(':', '-')}.csv`; link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000); notice();
}
function transportChanged() {
  const transport = $('#transport').value;
  $('#endpoint').disabled = transport === 'demo'; $('#baudLabel').hidden = transport !== 'serial';
  const defaults = {serial: '/dev/ttyACM0', udp: '0.0.0.0:14550', udpout: '127.0.0.1:14550', tcp: '127.0.0.1:5760', demo: ''};
  $('#endpoint').value = defaults[transport];
  const hints = {
    serial: '串口位于运行 Python 服务的电脑上。请关闭占用该串口的 QGC。',
    udp: '监听本机 UDP 端口。将飞控或路由器的遥测目标设为本机 IP 和这个端口。',
    udpout: '主动连接远端 UDP 端点；GCS 心跳可帮助远端识别返回地址。',
    tcp: '连接提供 MAVLink 字节流的 TCP 服务，例如 SITL 或 mavlink-router。',
    demo: '演示数据经过真实 MAVLink 编码与解析，不连接飞控。',
  };
  $('#sourceHint').textContent = hints[transport];
}
async function connect(demo = false) {
  $('#connectButton').disabled = true; notice();
  try {
    if (demo) { $('#transport').value = 'demo'; transportChanged(); }
    await api('/api/connect', {transport: $('#transport').value, endpoint: $('#endpoint').value,
      baud: Number($('#baud').value), heartbeat: $('#heartbeat').checked});
  } catch (error) { notice(error.message); }
  finally { $('#connectButton').disabled = false; }
}
async function refreshPorts() {
  try {
    const payload = await api('/api/ports'); $('#serialPorts').replaceChildren();
    for (const port of payload.ports) $('#serialPorts').append(new Option(port.description, port.device));
    if ($('#transport').value === 'serial' && payload.ports.length && $('#endpoint').value === '/dev/ttyACM0') $('#endpoint').value = payload.ports[0].device;
    if (!payload.dependencies) notice('请先安装解析依赖：pip install -r WebUI/requirements-mavlink.txt');
  } catch (error) { notice(error.message); }
}
$('#connectionForm').onsubmit = event => { event.preventDefault(); connect(); };
$('#transport').onchange = transportChanged;
$('#demoButton').onclick = () => connect(true);
$('#refreshPorts').onclick = refreshPorts;
$('#disconnectButton').onclick = async () => { try { await api('/api/disconnect', {}); } catch (error) { notice(error.message); } };
$('#openConnection').onclick = () => { $('#connectionPanel').hidden = !$('#connectionPanel').hidden; };
$('#requestStream').onclick = async () => {
  try { await api('/api/request-stream', {system: Number($('#targetSystem').value), component: Number($('#targetComponent').value), hz: Number($('#streamHz').value)}); notice(); }
  catch (error) { notice(error.message); }
};
for (const id of ['targetSystem', 'targetComponent']) $(`#${id}`).oninput = () => { $('#targetSystem').dataset.edited = 'true'; };
$('#fieldSearch').oninput = event => { state.search = event.target.value; renderFields(); };
$('#sourceFilter').onchange = event => { state.source = event.target.value; renderFields(); };
for (const button of document.querySelectorAll('[data-filter]')) button.onclick = () => {
  state.filter = button.dataset.filter;
  for (const item of document.querySelectorAll('[data-filter]')) item.classList.toggle('active', item === button);
  renderFields();
};
$('#addChart').onclick = () => { const chart = makeChart('自定义图表'); state.active = chart.id; $('#preset').value = 'custom'; renderCharts(); renderFields(); saveLayout(); chartElements.get(chart.id).scrollIntoView({behavior: 'smooth', block: 'nearest'}); };
$('#clearSelection').onclick = () => {
  const chart = state.charts.find(item => item.id === state.active);
  if (chart) { chart.traces = []; chart.wanted = null; renderLegend(chart); renderFields(); saveLayout(); }
};
$('#preset').onchange = event => { if (event.target.value !== 'custom') setPreset(event.target.value); };
$('#timeWindow').onchange = event => { state.window = Number(event.target.value); saveLayout(); };
$('#pauseButton').onclick = () => { if (state.paused) resume(); else { state.end = now(); state.paused = true; updatePause(); } };
$('#resetView').onclick = resume;
$('#exportCsv').onclick = exportCsv;
document.addEventListener('keydown', event => {
  if (['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement?.tagName)) return;
  if (event.key === '/') { event.preventDefault(); $('#fieldSearch').focus(); }
});
try {
  const saved = JSON.parse(localStorage.getItem('rlflight-mavlink-layout-v1') || 'null');
  if (saved?.charts?.length && saved.charts.length <= 20) {
    for (const item of saved.charts) {
      const chart = makeChart(String(item.title || '自定义图表').slice(0, 60), Array.isArray(item.wanted) ? item.wanted : null);
      chart.traces = Array.isArray(item.traces) ? item.traces.filter(trace => typeof trace.id === 'string').map(trace => ({id: trace.id, side: trace.side === 'right' ? 'right' : 'left'})).slice(0, 64) : [];
      chart.min = Number.isFinite(item.min) ? item.min : null; chart.max = Number.isFinite(item.max) ? item.max : null;
      if (chart.min !== null && chart.max !== null && chart.min >= chart.max) chart.min = chart.max = null;
    }
    state.window = Math.max(1, Math.min(300, Number(saved.window) || 30));
    $('#timeWindow').value = String(state.window); $('#preset').value = saved.preset || 'custom';
    state.active = state.charts[0].id; renderCharts();
  } else setPreset('compare');
} catch { setPreset('compare'); }
transportChanged(); refreshPorts(); poll(); requestAnimationFrame(animate);
