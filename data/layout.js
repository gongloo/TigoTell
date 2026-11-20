'use strict';

// ---------------------------------------------------------------------------
// Constants and state
// ---------------------------------------------------------------------------

const NS = 'http://www.w3.org/2000/svg';
const GRID = 10;
const STALE_MS = 60000;
const PANEL_W = 100;
const PANEL_H = 170;
const HEAT = ['#1e2a36', '#3f6212', '#6dad12', '#c6f08f'];

const METRICS = {
  power: { label: 'Power', unit: 'W', digits: 0, fromZero: true },
  voltage_in: { label: 'Voltage in', unit: 'V', digits: 1, fromZero: true },
  voltage_out: { label: 'Voltage out', unit: 'V', digits: 1, fromZero: true },
  current_in: { label: 'Current', unit: 'A', digits: 2, fromZero: true },
  temperature: { label: 'Temperature', unit: '°C', digits: 1, fromZero: false },
  rssi: { label: 'Signal (RSSI)', unit: 'dBm', digits: 0, fromZero: false },
  duty_cycle: { label: 'Duty cycle', unit: '', digits: 0, fromZero: true },
};

// Distinct from the green power ramp and from each other.
const STRING_COLORS = ['#38bdf8', '#fb923c', '#f472b6', '#facc15', '#a78bfa', '#2dd4bf', '#f87171', '#e7e5e4'];

const STRING_FIELDS = {
  power: 'Power',
  voltage: 'Voltage (sum)',
  active: 'Reporting panels',
  count: 'Panels',
};

const SYSTEM_FIELDS = {
  total_power: 'Total power',
  active_nodes: 'Active modules',
  known_nodes: 'Known modules',
  crc_errors: 'CRC errors',
  uptime: 'Uptime',
};

const GROUP_FIELDS = {
  'group.power': 'Power of panels inside',
  'group.active': 'Reporting panels inside',
  'group.count': 'Panels inside',
};

const state = {
  layout: { v: 1, st: [], el: [] },
  saved: '',            // last saved layout JSON, to detect changes
  maxBytes: 8192,
  live: { nodes: new Map(), system: {} },
  editing: false,
  tool: 'select',
  selected: new Set(),  // element ids
  view: { x: 0, y: 0, k: 1 },
  snap: true,
  undo: [],
  redo: [],
  armedNode: null,      // module waiting to be assigned to a panel
  armedString: null,    // string whose panels are being picked
  highlightString: null, // view mode: string shown in focus
  stringsCollapsed: true,
  detailsPanel: null,   // panel (element id) shown in the details card (view mode)
  metric: 'power',
  loaded: false,
  sim: { on: false, sun: 75, data: null, timer: null },
};

let nextId = 1;
let liveBusy = false;
const $ = (id) => document.getElementById(id);

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

function svgEl(tag, attrs = {}, parent) {
  const el = document.createElementNS(NS, tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (v !== undefined && v !== null) el.setAttribute(k, v);
  }
  if (parent) parent.appendChild(el);
  return el;
}

function snap(v, ev) {
  return state.snap && !(ev && ev.altKey) ? Math.round(v / GRID) * GRID : Math.round(v);
}

function fmt(value, digits) {
  if (value === undefined || value === null || Number.isNaN(value)) return '–';
  return value.toLocaleString(undefined, { minimumFractionDigits: digits, maximumFractionDigits: digits });
}

function fmtDuration(s) {
  if (s === undefined) return '–';
  const d = Math.floor(s / 86400);
  const h = Math.floor((s % 86400) / 3600);
  const m = Math.floor((s % 3600) / 60);
  return d > 0 ? `${d}d ${h}h` : h > 0 ? `${h}h ${m}m` : `${m}m`;
}

function mixColor(a, b, t) {
  const pa = parseInt(a.slice(1), 16);
  const pb = parseInt(b.slice(1), 16);
  const ch = (shift) => Math.round(((pa >> shift) & 255) * (1 - t) + ((pb >> shift) & 255) * t);
  return `rgb(${ch(16)},${ch(8)},${ch(0)})`;
}

function heatColor(t) {
  t = Math.max(0, Math.min(1, t));
  const seg = t * (HEAT.length - 1);
  const i = Math.min(HEAT.length - 2, Math.floor(seg));
  return mixColor(HEAT[i], HEAT[i + 1], seg - i);
}

function rotatePoint(px, py, cx, cy, deg) {
  const r = (deg * Math.PI) / 180;
  const c = Math.cos(r);
  const s = Math.sin(r);
  const dx = px - cx;
  const dy = py - cy;
  return { x: cx + dx * c - dy * s, y: cy + dx * s + dy * c };
}

let toastTimer;
function toast(msg, isError = false) {
  const t = $('toast');
  t.textContent = msg;
  t.className = 'toast show' + (isError ? ' error' : '');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => (t.className = 'toast'), isError ? 6000 : 3000);
}

function hint(msg) {
  const h = $('hint');
  h.textContent = msg || '';
  h.classList.toggle('show', !!msg);
}

function byId(id) {
  return state.layout.el.find((e) => e._id === id);
}

// Data the drawing shows: real live data, or simulated data in simulation mode.
function shown() {
  return state.sim.on && state.sim.data ? state.sim.data : state.live;
}

// Module data for a panel. In simulation, unlinked panels get data too, so
// the whole layout can be previewed before modules are assigned.
function panelNode(e) {
  if (e.n !== undefined) return shown().nodes.get(e.n) || null;
  return state.sim.on && state.sim.data ? state.sim.data.nodes.get(`s${e._id}`) || null : null;
}

function stringById(id) {
  return state.layout.st.find((x) => x.id === id) || null;
}

function stringStats(id) {
  let power = 0, voltage = 0, active = 0, count = 0;
  for (const e of state.layout.el) {
    if (e.t !== 'p' || e.g !== id) continue;
    count++;
    const n = panelNode(e);
    if (n && n.fresh) {
      power += n.power;
      voltage += n.voltage_out;
      active++;
    }
  }
  return { power, voltage, active, count };
}

// The metric whose number is printed on panels ("string" coloring prints power).
function valueMetric() {
  return state.metric === 'string' ? 'power' : state.metric;
}

function selectedEls() {
  return state.layout.el.filter((e) => state.selected.has(e._id));
}

// Serialised layout without runtime ids.
function layoutJson() {
  const out = { v: 1 };
  if (state.layout.st.length) out.st = state.layout.st;
  out.el = state.layout.el.map(({ _id, ...rest }) => rest);
  return JSON.stringify(out);
}

function setLayout(obj) {
  const el = Array.isArray(obj && obj.el) ? obj.el : [];
  const st = Array.isArray(obj && obj.st) ? obj.st : [];
  state.layout = {
    v: 1,
    st: st.filter((x) => x && Number.isInteger(x.id)).map((x) => ({ id: x.id, n: String(x.n || `String ${x.id}`), c: x.c || STRING_COLORS[0] })),
    el: el.filter((e) => e && typeof e === 'object' && ['p', 'r', 'l', 'x'].includes(e.t)),
  };
  state.layout.el.forEach((e) => (e._id = nextId++));
  state.selected.clear();
}

// ---------------------------------------------------------------------------
// Live data
// ---------------------------------------------------------------------------

// Browsers allow only a few parallel connections per device. A request the
// device never answers must time out, or later requests (and other pages of
// the device) queue up behind it.
async function fetchWithTimeout(url, options = {}, ms = 5000) {
  const ctrl = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), ms);
  try {
    return await fetch(url, { ...options, signal: ctrl.signal });
  } finally {
    clearTimeout(timer);
  }
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

// Polls one request at a time: the next one starts 5 s after the last ended.
async function pollLive() {
  await fetchLive();
  setTimeout(pollLive, 5000);
}

async function fetchLive() {
  if (liveBusy) return;
  liveBusy = true;
  try {
    const res = await fetchWithTimeout('/json', {}, 4500);
    if (!res.ok) throw new Error(res.statusText);
    const data = await res.json();
    const nodes = new Map();
    for (const n of data.nodes || []) {
      nodes.set(n.pv_node_id, { id: n.pv_node_id, barcode: n.barcode || '', fresh: false });
    }
    let total = 0;
    let active = 0;
    for (const p of data.power || []) {
      const age = data.uptime_ms - (p.last_updated || 0);
      const fresh = age <= STALE_MS;
      const entry = nodes.get(p.pv_node_id) || { id: p.pv_node_id, barcode: '' };
      Object.assign(entry, {
        voltage_in: p.voltage_in,
        voltage_out: p.voltage_out,
        current_in: p.current_in,
        temperature: p.temperature,
        rssi: p.rssi,
        duty_cycle: p.duty_cycle,
        gateway: p.gateway,
        power: p.voltage_out * p.current_in,
        age_s: Math.round(age / 1000),
        fresh,
      });
      nodes.set(p.pv_node_id, entry);
      if (fresh) {
        total += entry.power;
        active++;
      }
    }
    state.live = {
      nodes,
      system: {
        total_power: total,
        active_nodes: active,
        known_nodes: nodes.size,
        crc_errors: data.stats ? data.stats.crc_errors : undefined,
        uptime: data.uptime_ms !== undefined ? Math.floor(data.uptime_ms / 1000) : undefined,
      },
    };
    const totalEl = $('total');
    totalEl.textContent = `${fmt(total, 0)} W`;
    totalEl.classList.toggle('producing', total > 0);
    render();
    if (state.editing) {
      renderModules();
      // Avoid rebuilding while the user types a string name.
      if (!$('pane-strings').contains(document.activeElement)) renderStringsPane();
    }
    renderDetails();
  } catch (err) {
    console.error('Failed to fetch live data', err);
  } finally {
    liveBusy = false;
  }
}

// ---------------------------------------------------------------------------
// Value placeholders: {total_power}, {node:12.power}, {group.power}
// ---------------------------------------------------------------------------

function groupStats(rect) {
  const cx = rect.x + rect.w / 2;
  const cy = rect.y + rect.h / 2;
  let power = 0;
  let active = 0;
  let count = 0;
  for (const e of state.layout.el) {
    if (e.t !== 'p') continue;
    // Panel centre in the rectangle's own (unrotated) frame.
    const p = rotatePoint(e.x + e.w / 2, e.y + e.h / 2, cx, cy, -(rect.r || 0));
    if (p.x < rect.x || p.x > rect.x + rect.w || p.y < rect.y || p.y > rect.y + rect.h) continue;
    count++;
    const n = panelNode(e);
    if (n && n.fresh) {
      power += n.power;
      active++;
    }
  }
  return { power, active, count };
}

function fillTemplate(text, rect) {
  if (!text || !text.includes('{')) return text || '';
  let group = null;
  return text.replace(/\{([^{}]+)\}/g, (match, key) => {
    key = key.trim();
    if (key.startsWith('group.')) {
      if (!rect) return match;
      group = group || groupStats(rect);
      const f = key.slice(6);
      return f === 'power' ? fmt(group.power, 0) : f === 'active' || f === 'count' ? String(group[f]) : match;
    }
    const sm = key.match(/^string:(\d+)\.(\w+)$/);
    if (sm) {
      if (!stringById(Number(sm[1])) || !(sm[2] in STRING_FIELDS)) return match;
      const st = stringStats(Number(sm[1]));
      return sm[2] === 'power' ? fmt(st.power, 0) : sm[2] === 'voltage' ? fmt(st.voltage, 1) : String(st[sm[2]]);
    }
    const m = key.match(/^node:(\d+)\.(\w+)$/);
    if (m) {
      const n = shown().nodes.get(Number(m[1]));
      const meta = METRICS[m[2]];
      if (!meta) return match;
      return n && n.fresh ? fmt(n[m[2]], meta.digits) : '–';
    }
    const sys = shown().system;
    if (key === 'total_power') return fmt(sys.total_power, 0);
    if (key === 'uptime') return fmtDuration(sys.uptime);
    if (key in SYSTEM_FIELDS) return sys[key] === undefined ? '–' : String(sys[key]);
    return match;
  });
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

function metricRange() {
  const metric = valueMetric();
  const meta = METRICS[metric];
  let min = Infinity;
  let max = -Infinity;
  for (const e of state.layout.el) {
    if (e.t !== 'p') continue;
    const n = panelNode(e);
    if (!n || !n.fresh) continue;
    min = Math.min(min, n[metric]);
    max = Math.max(max, n[metric]);
  }
  if (min === Infinity) return null;
  if (meta.fromZero) min = Math.min(0, min);
  return { min, max };
}

function cellPath(w, h) {
  // 6 x 10 cells for portrait, 10 x 6 for landscape.
  const cols = w <= h ? 6 : 10;
  const rows = w <= h ? 10 : 6;
  let d = '';
  for (let i = 1; i < cols; i++) {
    const x = -w / 2 + (w * i) / cols;
    d += `M${x} ${-h / 2}V${h / 2}`;
  }
  for (let i = 1; i < rows; i++) {
    const y = -h / 2 + (h * i) / rows;
    d += `M${-w / 2} ${y}H${w / 2}`;
  }
  return d;
}

function renderPanel(e, layer, range) {
  const cx = e.x + e.w / 2;
  const cy = e.y + e.h / 2;
  const str = e.g !== undefined ? stringById(e.g) : null;
  const dim = !state.editing && state.highlightString !== null && (!str || str.id !== state.highlightString);
  const g = svgEl('g', { 'data-id': e._id, class: 'panel', opacity: dim ? 0.22 : null }, layer);
  const body = svgEl('g', { transform: `translate(${cx} ${cy}) rotate(${e.r || 0})` }, g);

  const n = panelNode(e);
  const linked = e.n !== undefined || n !== null;
  let fill = '#292524';
  let t = 0;
  if (!linked) {
    fill = 'url(#hatch)';
  } else if (state.metric === 'string') {
    fill = str ? mixColor('#1c1917', str.c, n && n.fresh ? 0.8 : 0.4) : '#292524';
    t = str && n && n.fresh ? 0.8 : 0;
  } else if (n && n.fresh && range) {
    const span = range.max - range.min;
    t = span > 0 ? (n[state.metric] - range.min) / span : 0.5;
    fill = heatColor(t);
  }

  svgEl('rect', {
    class: 'panel-fill', x: -e.w / 2, y: -e.h / 2, width: e.w, height: e.h, rx: 3,
    fill, stroke: str ? str.c : '#a8a29e', 'stroke-width': str ? 4 : 2,
  }, body);
  if (str) {
    // Colored strip along the top edge keeps the string visible on any fill.
    const strip = Math.max(5, Math.min(e.w, e.h) * 0.09);
    svgEl('rect', { x: -e.w / 2, y: -e.h / 2, width: e.w, height: strip, fill: str.c, 'pointer-events': 'none' }, body);
  }
  svgEl('path', {
    d: cellPath(e.w, e.h), stroke: t > 0.65 ? 'rgba(28,25,23,0.25)' : 'rgba(255,255,255,0.08)',
    'stroke-width': 1, fill: 'none', 'pointer-events': 'none',
  }, body);

  // Text stays upright regardless of rotation.
  const size = Math.max(9, Math.min(e.w, e.h) * 0.24);
  const ink = t > 0.65 ? '#1c1917' : '#fafaf9';
  if (linked) {
    const vm = valueMetric();
    const meta = METRICS[vm];
    const value = n && n.fresh ? `${fmt(n[vm], meta.digits)}${meta.unit ? ' ' + meta.unit : ''}` : '–';
    const v = svgEl('text', {
      class: 'panel-value', x: cx, y: cy + size * 0.35, 'text-anchor': 'middle',
      'font-size': size, fill: ink,
    }, g);
    v.textContent = value;
    const idText = svgEl('text', {
      class: 'panel-id', x: cx, y: cy - size * 0.9, 'text-anchor': 'middle',
      'font-size': size * 0.55, fill: ink, opacity: 0.7,
    }, g);
    idText.textContent = e.n !== undefined ? `#${e.n}` : 'unlinked';
  } else if (state.editing) {
    const q = svgEl('text', {
      class: 'panel-id', x: cx, y: cy + size * 0.35, 'text-anchor': 'middle',
      'font-size': size * 0.7, fill: '#a8a29e',
    }, g);
    q.textContent = 'unlinked';
  }
  if (state.armedString !== null && state.editing) {
    const as = stringById(state.armedString);
    svgEl('rect', {
      x: e.x - 3, y: e.y - 3, width: e.w + 6, height: e.h + 6, rx: 5, fill: 'none',
      stroke: as ? as.c : '#fff', 'stroke-width': 1.5, 'stroke-dasharray': '6 4', opacity: e.g === state.armedString ? 0 : 0.5,
      transform: `rotate(${e.r || 0} ${cx} ${cy})`, 'pointer-events': 'none',
    }, g);
  }
  if (state.armedNode !== null && state.editing) {
    // Make target panels obvious while a module is waiting to be assigned.
    svgEl('rect', {
      x: e.x - 3, y: e.y - 3, width: e.w + 6, height: e.h + 6, rx: 5, fill: 'none',
      stroke: '#6dad12', 'stroke-width': 1.5, 'stroke-dasharray': '6 4', opacity: 0.6,
      transform: `rotate(${e.r || 0} ${cx} ${cy})`, 'pointer-events': 'none',
    }, g);
  }
}

function renderRect(e, layer) {
  const cx = e.x + e.w / 2;
  const cy = e.y + e.h / 2;
  const g = svgEl('g', { 'data-id': e._id, transform: `rotate(${e.r || 0} ${cx} ${cy})` }, layer);
  svgEl('rect', {
    x: e.x, y: e.y, width: e.w, height: e.h, rx: 4,
    fill: e.f || '#6dad12', 'fill-opacity': (e.op ?? 12) / 100,
    stroke: e.s || '#78716c', 'stroke-width': e.sw ?? 2,
  }, g);
  if (e.l) {
    const fs = e.fs || 16;
    const lines = fillTemplate(e.l, e).split('\n');
    const txt = svgEl('text', { x: e.x + 10, y: e.y + 10 + fs, 'font-size': fs, fill: e.c || '#d6d3d1', 'font-weight': 600 }, g);
    lines.forEach((line, i) => {
      const span = svgEl('tspan', { x: e.x + 10, dy: i === 0 ? 0 : fs * 1.25 }, txt);
      span.textContent = line;
    });
  }
}

function renderLine(e, layer) {
  const g = svgEl('g', { 'data-id': e._id }, layer);
  svgEl('line', {
    x1: e.x, y1: e.y, x2: e.x2, y2: e.y2, stroke: e.s || '#a8a29e', 'stroke-width': e.sw ?? 3,
    'stroke-linecap': 'round', 'stroke-dasharray': e.d ? `${(e.sw ?? 3) * 3} ${(e.sw ?? 3) * 2}` : null,
  }, g);
  // Wide invisible stroke makes thin lines easy to grab.
  svgEl('line', { x1: e.x, y1: e.y, x2: e.x2, y2: e.y2, stroke: 'transparent', 'stroke-width': 16 }, g);
}

function renderText(e, layer) {
  const fs = e.fs || 18;
  const g = svgEl('g', { 'data-id': e._id, transform: e.r ? `rotate(${e.r} ${e.x} ${e.y})` : null }, layer);
  const txt = svgEl('text', {
    x: e.x, y: e.y, 'font-size': fs, fill: e.c || '#fafaf9',
    'font-weight': e.b ? 700 : 400, 'text-anchor': e.a || 'start',
    style: 'font-variant-numeric: tabular-nums',
  }, g);
  fillTemplate(e.txt || '', null).split('\n').forEach((line, i) => {
    const span = svgEl('tspan', { x: e.x, dy: i === 0 ? 0 : fs * 1.25 }, txt);
    span.textContent = line || ' ';
  });
}

function render() {
  // New or relinked panels need simulated data right away, not on the next tick.
  if (state.sim.on && state.layout.el.some((e) => e.t === 'p' && !state.sim.data.nodes.has(e.n !== undefined ? e.n : `s${e._id}`))) {
    simulate();
  }
  const layer = $('layer-content');
  layer.replaceChildren();
  const range = metricRange();
  for (const e of state.layout.el) {
    if (e.t === 'p') renderPanel(e, layer, range);
    else if (e.t === 'r') renderRect(e, layer);
    else if (e.t === 'l') renderLine(e, layer);
    else if (e.t === 'x') renderText(e, layer);
  }
  renderOverlay();
  renderLegend(range);
  renderStringsCard();
  $('empty').classList.toggle('show', state.loaded && !state.editing && state.layout.el.length === 0);
}

function renderOverlay() {
  const layer = $('layer-overlay');
  layer.replaceChildren();
  const k = state.view.k;
  if (!state.editing) {
    const e = state.detailsPanel !== null ? byId(state.detailsPanel) : null;
    if (e) {
      const cx = e.x + e.w / 2;
      const cy = e.y + e.h / 2;
      svgEl('rect', {
        x: e.x - 6 / k, y: e.y - 6 / k, width: e.w + 12 / k, height: e.h + 12 / k, rx: 6 / k, fill: 'none',
        stroke: '#fafaf9', 'stroke-width': 2.5 / k, transform: `rotate(${e.r || 0} ${cx} ${cy})`, 'pointer-events': 'none',
      }, layer);
    }
    return;
  }
  const handleSize = 10 / k;
  const single = state.selected.size === 1 ? selectedEls()[0] : null;

  for (const e of selectedEls()) {
    if (e.t === 'p' || e.t === 'r') {
      const cx = e.x + e.w / 2;
      const cy = e.y + e.h / 2;
      svgEl('rect', {
        x: e.x - 4 / k, y: e.y - 4 / k, width: e.w + 8 / k, height: e.h + 8 / k, fill: 'none',
        stroke: '#b4eb82', 'stroke-width': 1.5 / k, 'stroke-dasharray': `${5 / k} ${4 / k}`,
        transform: `rotate(${e.r || 0} ${cx} ${cy})`, 'pointer-events': 'none',
      }, layer);
      if (single === e) {
        for (const [name, sx, sy] of [['nw', -1, -1], ['ne', 1, -1], ['se', 1, 1], ['sw', -1, 1]]) {
          const p = rotatePoint(cx + (sx * e.w) / 2, cy + (sy * e.h) / 2, cx, cy, e.r || 0);
          svgEl('rect', {
            class: `handle ${name}`, 'data-handle': name, x: p.x - handleSize / 2, y: p.y - handleSize / 2,
            width: handleSize, height: handleSize, fill: '#1c1917', stroke: '#b4eb82', 'stroke-width': 1.5 / k,
          }, layer);
        }
      }
    } else if (e.t === 'l') {
      svgEl('line', {
        x1: e.x, y1: e.y, x2: e.x2, y2: e.y2, stroke: '#b4eb82', 'stroke-opacity': 0.35,
        'stroke-width': (e.sw ?? 3) + 8 / k, 'stroke-linecap': 'round', 'pointer-events': 'none',
      }, layer);
      if (single === e) {
        for (const [name, x, y] of [['p1', e.x, e.y], ['p2', e.x2, e.y2]]) {
          svgEl('circle', {
            class: 'handle pt', 'data-handle': name, cx: x, cy: y, r: handleSize / 1.6,
            fill: '#1c1917', stroke: '#b4eb82', 'stroke-width': 1.5 / k,
          }, layer);
        }
      }
    } else if (e.t === 'x') {
      const node = document.querySelector(`#layer-content [data-id="${e._id}"]`);
      if (!node) continue;
      const b = node.getBBox();
      svgEl('rect', {
        x: b.x - 4 / k, y: b.y - 2 / k, width: b.width + 8 / k, height: b.height + 4 / k, fill: 'none',
        stroke: '#b4eb82', 'stroke-width': 1.5 / k, 'stroke-dasharray': `${5 / k} ${4 / k}`, 'pointer-events': 'none',
        transform: e.r ? `rotate(${e.r} ${e.x} ${e.y})` : null,
      }, layer);
    }
  }

  if (drag && drag.type === 'marquee') {
    const r = drag.rect;
    svgEl('rect', {
      x: r.x, y: r.y, width: r.w, height: r.h, fill: 'rgba(109,173,18,0.1)',
      stroke: '#6dad12', 'stroke-width': 1 / k, 'pointer-events': 'none',
    }, layer);
  }
  if (drag && drag.preview) {
    const p = drag.preview;
    if (p.t === 'r') {
      svgEl('rect', { x: p.x, y: p.y, width: p.w, height: p.h, fill: 'rgba(109,173,18,0.12)', stroke: '#6dad12', 'stroke-width': 2 / k }, layer);
    } else if (p.t === 'l') {
      svgEl('line', { x1: p.x, y1: p.y, x2: p.x2, y2: p.y2, stroke: '#6dad12', 'stroke-width': 3, 'stroke-linecap': 'round' }, layer);
    }
  }
}

function renderLegend(range) {
  if (state.metric === 'string') {
    $('legend').style.display = 'none';
    $('legend').classList.add('off');
    return;
  }
  $('legend').classList.remove('off');
  const meta = METRICS[state.metric];
  $('legend-title').textContent = `${meta.label}${meta.unit ? ' (' + meta.unit + ')' : ''}`;
  $('legend-min').textContent = range ? fmt(range.min, meta.digits) : '–';
  $('legend-max').textContent = range ? fmt(range.max, meta.digits) : '–';
  const hasPanels = state.layout.el.some((e) => e.t === 'p' && (e.n !== undefined || state.sim.on));
  $('legend').style.display = hasPanels ? '' : 'none';
}

function applyView() {
  const { x, y, k } = state.view;
  $('world').setAttribute('transform', `translate(${x} ${y}) scale(${k})`);
  const canvas = $('canvas');
  const grid = GRID * 2 * k;
  canvas.style.setProperty('--grid-size', `${grid}px`);
  canvas.style.setProperty('--grid-x', `${x}px`);
  canvas.style.setProperty('--grid-y', `${y}px`);
}

// ---------------------------------------------------------------------------
// View: zoom, pan, fit
// ---------------------------------------------------------------------------

function toWorld(clientX, clientY) {
  const r = $('canvas').getBoundingClientRect();
  return { x: (clientX - r.left - state.view.x) / state.view.k, y: (clientY - r.top - state.view.y) / state.view.k };
}

function zoomAt(factor, clientX, clientY) {
  const r = $('canvas').getBoundingClientRect();
  const px = clientX - r.left;
  const py = clientY - r.top;
  const k = Math.max(0.1, Math.min(8, state.view.k * factor));
  state.view.x = px - ((px - state.view.x) * k) / state.view.k;
  state.view.y = py - ((py - state.view.y) * k) / state.view.k;
  state.view.k = k;
  applyView();
  renderOverlay();
}

function zoomCenter(factor) {
  const r = $('canvas').getBoundingClientRect();
  zoomAt(factor, r.left + r.width / 2, r.top + r.height / 2);
}

function fitView() {
  const r = $('canvas').getBoundingClientRect();
  if (r.width === 0) return;
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  const content = $('layer-content');
  if (state.layout.el.length > 0 && content.childNodes.length > 0) {
    const b = content.getBBox();
    minX = b.x; minY = b.y; maxX = b.x + b.width; maxY = b.y + b.height;
  }
  if (minX === Infinity) {
    state.view = { x: r.width / 2, y: r.height / 2, k: 1 };
  } else {
    const narrow = r.width <= 800;
    const pad = narrow ? 12 : 40;
    // On phones the drawing starts right below the strings card instead of
    // being centred vertically, so no empty band is left above it.
    const card = $('strings-card');
    const top = narrow && card.classList.contains('show') ? card.offsetTop + card.offsetHeight + pad : pad;
    const k = Math.max(0.1, Math.min(3, Math.min((r.width - pad * 2) / (maxX - minX || 1), (r.height - top - pad) / (maxY - minY || 1))));
    state.view = {
      k,
      x: (r.width - (maxX - minX) * k) / 2 - minX * k,
      y: narrow ? top - minY * k : (r.height - (maxY - minY) * k) / 2 - minY * k,
    };
  }
  applyView();
  renderOverlay();
}

// ---------------------------------------------------------------------------
// Editing: history, mutations
// ---------------------------------------------------------------------------

function checkpoint() {
  state.undo.push(layoutJson());
  if (state.undo.length > 100) state.undo.shift();
  state.redo = [];
  updateToolbar();
}

function restore(json) {
  const ids = [...state.selected];
  setLayout(JSON.parse(json));
  renderStringsPane();
  // Keep selection where it still makes sense (same index positions).
  state.selected = new Set(ids.filter((id) => byId(id)));
  render();
  renderProps();
  renderModules();
  updateToolbar();
}

function undo() {
  if (!state.undo.length) return;
  state.redo.push(layoutJson());
  restore(state.undo.pop());
}

function redo() {
  if (!state.redo.length) return;
  state.undo.push(layoutJson());
  restore(state.redo.pop());
}

function changed() {
  render();
  renderProps();
  renderModules();
  renderStringsPane();
  updateToolbar();
}

function updateToolbar() {
  $('undo-btn').disabled = state.undo.length === 0;
  $('redo-btn').disabled = state.redo.length === 0;
  $('snap-btn').classList.toggle('on', state.snap);
}

function addElement(e) {
  e._id = nextId++;
  state.layout.el.push(e);
  return e;
}

function deleteSelected() {
  if (!state.selected.size) return;
  checkpoint();
  state.layout.el = state.layout.el.filter((e) => !state.selected.has(e._id));
  state.selected.clear();
  changed();
}

function duplicateSelected() {
  if (!state.selected.size) return;
  checkpoint();
  const copies = selectedEls().map((e) => {
    const { _id, ...rest } = e;
    const c = JSON.parse(JSON.stringify(rest));
    c.x += GRID * 2;
    c.y += GRID * 2;
    if (c.t === 'l') { c.x2 += GRID * 2; c.y2 += GRID * 2; }
    delete c.n; // a module belongs to exactly one panel
    return addElement(c);
  });
  state.selected = new Set(copies.map((c) => c._id));
  changed();
}

function reorderSelected(toFront) {
  if (!state.selected.size) return;
  checkpoint();
  const sel = state.layout.el.filter((e) => state.selected.has(e._id));
  const rest = state.layout.el.filter((e) => !state.selected.has(e._id));
  state.layout.el = toFront ? [...rest, ...sel] : [...sel, ...rest];
  changed();
}

function rotateSelected(deg) {
  const els = selectedEls().filter((e) => e.t !== 'l');
  if (!els.length) return;
  checkpoint();
  for (const e of els) e.r = (((e.r || 0) + deg) % 360 + 360) % 360 || undefined;
  changed();
}

function moveSelected(dx, dy) {
  for (const e of selectedEls()) {
    e.x += dx;
    e.y += dy;
    if (e.t === 'l') { e.x2 += dx; e.y2 += dy; }
  }
}

function assignNode(panel, nodeId) {
  checkpoint();
  for (const e of state.layout.el) {
    if (e.t === 'p' && e.n === nodeId) delete e.n;
  }
  if (nodeId === undefined) delete panel.n;
  else panel.n = nodeId;
  changed();
}

// ---------------------------------------------------------------------------
// Creating elements
// ---------------------------------------------------------------------------

function createAt(tool, p, ev) {
  if (tool === 'panel') {
    checkpoint();
    const e = addElement({ t: 'p', x: snap(p.x - PANEL_W / 2, ev), y: snap(p.y - PANEL_H / 2, ev), w: PANEL_W, h: PANEL_H });
    state.selected = new Set([e._id]);
  } else if (tool === 'text') {
    checkpoint();
    const e = addElement({ t: 'x', x: snap(p.x, ev), y: snap(p.y, ev), txt: 'Text', fs: 18 });
    state.selected = new Set([e._id]);
    setTool('select');
    changed();
    const input = document.querySelector('#pane-props textarea');
    if (input) { input.focus(); input.select(); }
    return;
  } else if (tool === 'array') {
    openArrayDialog(p, ev);
    return;
  }
  setTool('select');
  changed();
}

function openArrayDialog(p, ev) {
  const dlg = $('array-dialog');
  dlg.returnValue = '';
  dlg.onclose = () => {
    if (dlg.returnValue !== 'ok') return;
    const cols = Math.max(1, Math.min(40, parseInt($('arr-cols').value) || 1));
    const rows = Math.max(1, Math.min(20, parseInt($('arr-rows').value) || 1));
    const gap = Math.max(0, parseInt($('arr-gap').value) || 0);
    const landscape = $('arr-orient').value === 'landscape';
    const w = landscape ? PANEL_H : PANEL_W;
    const h = landscape ? PANEL_W : PANEL_H;
    const x0 = snap(p.x, ev);
    const y0 = snap(p.y, ev);
    checkpoint();
    const ids = [];
    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < cols; c++) {
        ids.push(addElement({ t: 'p', x: x0 + c * (w + gap), y: y0 + r * (h + gap), w, h })._id);
      }
    }
    state.selected = new Set(ids);
    setTool('select');
    changed();
  };
  dlg.showModal();
}

// ---------------------------------------------------------------------------
// Pointer interaction
// ---------------------------------------------------------------------------

const pointers = new Map();
let drag = null;
let pinch = null;

function onPointerDown(ev) {
  const canvas = $('canvas');
  canvas.setPointerCapture(ev.pointerId);
  pointers.set(ev.pointerId, { x: ev.clientX, y: ev.clientY });

  if (pointers.size === 2) {
    // Second finger: switch to pinch zoom, abandon any drag.
    if (drag && drag.moved && drag.undoSaved) undo();
    drag = null;
    const [a, b] = [...pointers.values()];
    pinch = { dist: Math.hypot(a.x - b.x, a.y - b.y), cx: (a.x + b.x) / 2, cy: (a.y + b.y) / 2 };
    return;
  }
  if (pointers.size > 2) return;

  const p = toWorld(ev.clientX, ev.clientY);
  const target = ev.target.closest ? ev.target : null;
  const handle = target && target.getAttribute('data-handle');
  const hitNode = target && target.closest('[data-id]');
  const hit = hitNode ? byId(Number(hitNode.getAttribute('data-id'))) : null;
  const base = { startX: ev.clientX, startY: ev.clientY, start: p, moved: false, ev };

  if (!state.editing) {
    drag = { ...base, type: 'pan', view: { ...state.view }, click: hit };
    return;
  }

  if (state.armedNode !== null) {
    drag = { ...base, type: 'pan', view: { ...state.view }, assign: hit && hit.t === 'p' ? hit : null };
    return;
  }
  if (state.armedString !== null) {
    if (hit && hit.t === 'p') {
      // Click toggles a panel; dragging across panels adds them all.
      drag = { ...base, type: 'paint', first: hit, painted: new Set() };
    } else {
      drag = { ...base, type: 'pan', view: { ...state.view }, endStringPick: true };
    }
    return;
  }

  if (state.tool === 'rect' || state.tool === 'line') {
    drag = { ...base, type: 'draw', tool: state.tool };
    return;
  }
  if (state.tool !== 'select') {
    drag = { ...base, type: 'place', tool: state.tool };
    return;
  }

  if (handle && state.selected.size === 1) {
    const e = selectedEls()[0];
    drag = { ...base, type: 'handle', handle, el: e, orig: JSON.parse(JSON.stringify(e)) };
    return;
  }
  if (hit) {
    if (ev.shiftKey) {
      if (state.selected.has(hit._id)) state.selected.delete(hit._id);
      else state.selected.add(hit._id);
      renderOverlay();
      renderProps();
      drag = null;
      return;
    }
    if (!state.selected.has(hit._id)) {
      state.selected = new Set([hit._id]);
      renderOverlay();
      renderProps();
    }
    drag = { ...base, type: 'move', orig: selectedEls().map((e) => ({ e, x: e.x, y: e.y, x2: e.x2, y2: e.y2 })) };
    return;
  }
  if (ev.shiftKey || ev.pointerType === 'mouse' && ev.button === 0 && ev.ctrlKey) {
    drag = { ...base, type: 'marquee', rect: { x: p.x, y: p.y, w: 0, h: 0 } };
    return;
  }
  drag = { ...base, type: 'pan', view: { ...state.view }, clearOnClick: true };
}

function onPointerMove(ev) {
  if (!pointers.has(ev.pointerId)) return;
  pointers.set(ev.pointerId, { x: ev.clientX, y: ev.clientY });

  if (pinch && pointers.size === 2) {
    const [a, b] = [...pointers.values()];
    const dist = Math.hypot(a.x - b.x, a.y - b.y);
    const cx = (a.x + b.x) / 2;
    const cy = (a.y + b.y) / 2;
    state.view.x += cx - pinch.cx;
    state.view.y += cy - pinch.cy;
    zoomAt(dist / pinch.dist, cx, cy);
    pinch = { dist, cx, cy };
    return;
  }
  if (!drag) return;

  const dxScreen = ev.clientX - drag.startX;
  const dyScreen = ev.clientY - drag.startY;
  // Fingers wobble; a tap should still count as a tap.
  const slop = ev.pointerType === 'touch' ? 12 : 4;
  if (!drag.moved && Math.hypot(dxScreen, dyScreen) < slop) return;
  drag.moved = true;
  const p = toWorld(ev.clientX, ev.clientY);

  if (drag.type === 'paint') {
    if (!drag.undoSaved) { checkpoint(); drag.undoSaved = true; }
    const node = document.elementFromPoint(ev.clientX, ev.clientY);
    const gEl = node && node.closest && node.closest('#layer-content [data-id]');
    for (const e of [drag.first, gEl && byId(Number(gEl.getAttribute('data-id')))]) {
      if (e && e.t === 'p' && e.g !== state.armedString) {
        e.g = state.armedString;
        drag.painted.add(e._id);
      }
    }
    render();
  } else if (drag.type === 'pan') {
    state.view.x = drag.view.x + dxScreen;
    state.view.y = drag.view.y + dyScreen;
    applyView();
  } else if (drag.type === 'move') {
    if (!drag.undoSaved) { checkpoint(); drag.undoSaved = true; }
    const dx = snap(p.x - drag.start.x, ev);
    const dy = snap(p.y - drag.start.y, ev);
    for (const o of drag.orig) {
      o.e.x = o.x + dx;
      o.e.y = o.y + dy;
      if (o.e.t === 'l') { o.e.x2 = o.x2 + dx; o.e.y2 = o.y2 + dy; }
    }
    render();
  } else if (drag.type === 'handle') {
    if (!drag.undoSaved) { checkpoint(); drag.undoSaved = true; }
    resizeWithHandle(drag, p, ev);
    render();
  } else if (drag.type === 'marquee') {
    const x = Math.min(p.x, drag.start.x);
    const y = Math.min(p.y, drag.start.y);
    drag.rect = { x, y, w: Math.abs(p.x - drag.start.x), h: Math.abs(p.y - drag.start.y) };
    renderOverlay();
  } else if (drag.type === 'draw') {
    const s = { x: snap(drag.start.x, ev), y: snap(drag.start.y, ev) };
    const e = { x: snap(p.x, ev), y: snap(p.y, ev) };
    drag.preview = drag.tool === 'rect'
      ? { t: 'r', x: Math.min(s.x, e.x), y: Math.min(s.y, e.y), w: Math.abs(e.x - s.x), h: Math.abs(e.y - s.y) }
      : { t: 'l', x: s.x, y: s.y, x2: e.x, y2: e.y };
    renderOverlay();
  }
}

function onPointerUp(ev) {
  pointers.delete(ev.pointerId);
  if (pinch) {
    if (pointers.size < 2) pinch = null;
    return;
  }
  if (!drag) return;
  const d = drag;
  drag = null;
  const p = toWorld(ev.clientX, ev.clientY);

  if (d.type === 'paint') {
    if (!d.moved) {
      checkpoint();
      if (d.first.g === state.armedString) delete d.first.g;
      else d.first.g = state.armedString;
    }
    changed();
  } else if (d.type === 'pan' && !d.moved) {
    if (d.endStringPick) {
      armString(null);
    } else if (d.assign) {
      assignNode(d.assign, state.armedNode);
      toast(`Module ${state.armedNode} linked to panel`);
      armNode(null);
    } else if (state.armedNode !== null) {
      armNode(null);
    } else if (d.click && d.click.t === 'p') {
      state.detailsPanel = d.click._id;
      renderDetails();
      renderOverlay();
    } else if (!state.editing) {
      state.detailsPanel = null;
      renderDetails();
      renderOverlay();
      if (state.highlightString !== null) { state.highlightString = null; render(); }
    } else if (d.clearOnClick && state.selected.size) {
      state.selected.clear();
      renderOverlay();
      renderProps();
    }
  } else if (d.type === 'move' || d.type === 'handle') {
    if (d.moved) changed();
  } else if (d.type === 'marquee') {
    const r = d.rect;
    for (const e of state.layout.el) {
      const ex = e.t === 'l' ? Math.min(e.x, e.x2) : e.x;
      const ey = e.t === 'l' ? Math.min(e.y, e.y2) : e.y;
      const ew = e.t === 'l' ? Math.abs(e.x2 - e.x) : e.w || 0;
      const eh = e.t === 'l' ? Math.abs(e.y2 - e.y) : e.h || 0;
      if (ex >= r.x && ey >= r.y && ex + ew <= r.x + r.w && ey + eh <= r.y + r.h) state.selected.add(e._id);
    }
    renderOverlay();
    renderProps();
  } else if (d.type === 'draw') {
    checkpoint();
    let e;
    const pr = d.preview;
    if (d.tool === 'rect') {
      e = pr && pr.w >= GRID && pr.h >= GRID
        ? addElement({ t: 'r', x: pr.x, y: pr.y, w: pr.w, h: pr.h })
        : addElement({ t: 'r', x: snap(p.x, ev), y: snap(p.y, ev), w: 300, h: 200 });
    } else {
      e = pr && (pr.x !== pr.x2 || pr.y !== pr.y2)
        ? addElement({ t: 'l', x: pr.x, y: pr.y, x2: pr.x2, y2: pr.y2 })
        : addElement({ t: 'l', x: snap(p.x, ev), y: snap(p.y, ev), x2: snap(p.x, ev) + 200, y2: snap(p.y, ev) });
    }
    state.selected = new Set([e._id]);
    setTool('select');
    changed();
  } else if (d.type === 'place' && !d.moved) {
    createAt(d.tool, p, ev);
  }
}

function resizeWithHandle(d, p, ev) {
  const e = d.el;
  const o = d.orig;
  if (e.t === 'l') {
    if (d.handle === 'p1') { e.x = snap(p.x, ev); e.y = snap(p.y, ev); }
    else { e.x2 = snap(p.x, ev); e.y2 = snap(p.y, ev); }
    return;
  }
  // Resize in the element's own frame with the opposite corner fixed.
  const sx = d.handle.includes('e') ? 1 : -1;
  const sy = d.handle.includes('s') ? 1 : -1;
  const cx = o.x + o.w / 2;
  const cy = o.y + o.h / 2;
  const r = o.r || 0;
  const local = rotatePoint(p.x, p.y, cx, cy, -r);
  const lx = local.x - cx;
  const ly = local.y - cy;
  const fx = (-sx * o.w) / 2;
  const fy = (-sy * o.h) / 2;
  const w = Math.max(GRID, snap(Math.abs(lx - fx), ev));
  const h = Math.max(GRID, snap(Math.abs(ly - fy), ev));
  const mx = fx + (sx * w) / 2;
  const my = fy + (sy * h) / 2;
  const c = rotatePoint(cx + mx, cy + my, cx, cy, r);
  e.w = w;
  e.h = h;
  e.x = Math.round(c.x - w / 2);
  e.y = Math.round(c.y - h / 2);
}

// ---------------------------------------------------------------------------
// Properties panel
// ---------------------------------------------------------------------------

function field(label, input) {
  const wrap = document.createElement('label');
  wrap.className = 'field';
  const span = document.createElement('span');
  span.textContent = label;
  wrap.append(span, input);
  return wrap;
}

function input(type, value, onChange, attrs = {}) {
  const el = document.createElement(type === 'textarea' ? 'textarea' : 'input');
  if (type !== 'textarea') el.type = type;
  else Object.assign(el.style, { minHeight: '4rem', resize: 'vertical' });
  if (type === 'textarea') {
    el.className = 'ta';
    Object.assign(el.style, {
      padding: '0.4rem 0.6rem', background: 'rgba(255,255,255,0.05)', border: '1px solid rgba(255,255,255,0.12)',
      borderRadius: '0.5rem', color: 'var(--text)', font: 'inherit', fontSize: '0.875rem',
    });
  }
  for (const [k, v] of Object.entries(attrs)) el.setAttribute(k, v);
  if (type === 'checkbox') el.checked = !!value;
  else el.value = value ?? '';
  let saved = false;
  const handler = () => {
    if (type === 'number' && (el.value === '' || Number.isNaN(Number(el.value)))) return;
    // One undo step per editing session of a field.
    if (!saved) { checkpoint(); saved = true; }
    onChange(type === 'checkbox' ? el.checked : type === 'number' ? Number(el.value) : el.value);
    render();
    renderModules();
  };
  el.addEventListener(type === 'checkbox' || type === 'color' ? 'change' : 'input', handler);
  el.addEventListener('change', () => { saved = false; });
  return el;
}

function selectEl(options, value, onChange) {
  const sel = document.createElement('select');
  for (const o of options) {
    if (o.group) {
      const g = document.createElement('optgroup');
      g.label = o.group;
      for (const c of o.options) g.append(new Option(c.label, c.value, false, String(c.value) === String(value)));
      sel.append(g);
    } else {
      sel.append(new Option(o.label, o.value, false, String(o.value) === String(value)));
    }
  }
  sel.addEventListener('change', () => onChange(sel.value));
  return sel;
}

function geometryFields(e) {
  const grid = document.createElement('div');
  grid.className = 'grid4';
  const keys = e.t === 'l' ? [['x', 'X1'], ['y', 'Y1'], ['x2', 'X2'], ['y2', 'Y2']]
    : e.t === 'x' ? [['x', 'X'], ['y', 'Y']]
      : [['x', 'X'], ['y', 'Y'], ['w', 'Width'], ['h', 'Height']];
  for (const [k, label] of keys) {
    const l = document.createElement('label');
    l.append(label, input('number', e[k], (v) => { e[k] = k === 'w' || k === 'h' ? Math.max(GRID, v) : v; }, { step: GRID }));
    grid.append(l);
  }
  if (e.t !== 'l') {
    const l = document.createElement('label');
    l.append('Rotation (°)', input('number', e.r || 0, (v) => { e.r = ((v % 360) + 360) % 360 || undefined; }, { step: 15 }));
    grid.append(l);
  }
  return grid;
}

function valuePicker(target, key, withGroup) {
  // Two-step picker that inserts a {placeholder} into a text field.
  const wrap = document.createElement('div');
  wrap.className = 'field';
  const span = document.createElement('span');
  span.textContent = 'Insert live value';
  const row = document.createElement('div');
  row.className = 'row';

  const sources = [{ label: 'System', value: 'system' }];
  if (withGroup) sources.push({ label: 'Panels inside this rectangle', value: 'group' });
  for (const st of state.layout.st) sources.push({ label: st.n, value: `string:${st.id}` });
  const ids = [...state.live.nodes.keys()].sort((a, b) => a - b);
  for (const id of ids) sources.push({ label: `Module ${id}`, value: `node:${id}` });

  const fieldSel = document.createElement('select');
  const fillFields = (src) => {
    fieldSel.replaceChildren();
    const entries = src === 'system' ? Object.entries(SYSTEM_FIELDS)
      : src === 'group' ? Object.entries(GROUP_FIELDS)
        : src.startsWith('string:') ? Object.entries(STRING_FIELDS).map(([k, label]) => [`${src}.${k}`, label])
          : Object.entries(METRICS).map(([k, m]) => [`${src}.${k}`, m.label]);
    for (const [k, label] of entries) fieldSel.append(new Option(label, k));
  };
  const srcSel = selectEl(sources, 'system', fillFields);
  fillFields('system');

  const add = document.createElement('button');
  add.className = 'btn';
  add.type = 'button';
  add.textContent = 'Insert';
  add.onclick = () => {
    checkpoint();
    const key2 = fieldSel.value;
    const unit = key2 in SYSTEM_FIELDS || key2.startsWith('group.') || key2.startsWith('string:')
      ? (key2.endsWith('power') ? ' W' : key2.endsWith('voltage') ? ' V' : '')
      : (METRICS[key2.split('.').pop()].unit ? ' ' + METRICS[key2.split('.').pop()].unit : '');
    target[key] = `${target[key] || ''}{${key2}}${unit}`;
    changed();
  };
  row.append(srcSel, fieldSel);
  wrap.append(span, row);
  const row2 = document.createElement('div');
  row2.className = 'row';
  row2.style.marginTop = '0.4rem';
  row2.append(document.createElement('span'), add);
  wrap.append(row2);
  return wrap;
}

function actionsRow() {
  const row = document.createElement('div');
  row.className = 'actions';
  const btn = (label, fn, cls = '') => {
    const b = document.createElement('button');
    b.className = 'btn ' + cls;
    b.type = 'button';
    b.textContent = label;
    b.onclick = fn;
    row.append(b);
  };
  if (selectedEls().some((e) => e.t !== 'l')) btn('Rotate 90°', () => rotateSelected(90));
  btn('Duplicate', duplicateSelected);
  btn('To front', () => reorderSelected(true));
  btn('To back', () => reorderSelected(false));
  btn('Delete', deleteSelected, 'danger');
  return row;
}

function renderProps() {
  const pane = $('pane-props');
  if (!pane || !state.editing) return;
  pane.replaceChildren();
  const els = selectedEls();
  const h = document.createElement('h3');
  pane.append(h);

  if (els.length === 0) {
    h.textContent = 'Layout';
    const p = document.createElement('p');
    p.className = 'muted';
    p.innerHTML = 'Pick a tool on the left and click into the canvas. Drag empty space to pan, '
      + 'scroll or pinch to zoom, hold Shift and drag to select several elements.<br><br>'
      + 'Keys: Delete, Ctrl+Z / Ctrl+Shift+Z, Ctrl+D duplicate, arrows move, Esc ends a tool.';
    pane.append(p);
    const bytes = new Blob([layoutJson()]).size;
    const meter = document.createElement('div');
    meter.className = 'size-meter' + (bytes > state.maxBytes ? ' full' : '');
    const bar = document.createElement('div');
    bar.style.width = `${Math.min(100, (bytes / state.maxBytes) * 100)}%`;
    meter.append(bar);
    const label = document.createElement('p');
    label.className = 'muted';
    label.textContent = `Device storage: ${(bytes / 1024).toFixed(1)} of ${(state.maxBytes / 1024).toFixed(1)} KB available for the layout used`;
    const space = document.createElement('div');
    space.style.marginTop = '1.25rem';
    space.append(meter, label);
    pane.append(space);
    return;
  }

  if (els.length > 1) {
    h.textContent = `${els.length} elements`;
    const panels = els.filter((e) => e.t === 'p');
    if (panels.length) {
      const p = document.createElement('p');
      p.className = 'muted';
      p.textContent = `${panels.length} panels, ${panels.filter((e) => e.n !== undefined).length} linked to a module.`;
      p.style.marginBottom = '0.9rem';
      pane.append(p, field('String for selected panels', stringSelect(panels)));
    }
    pane.append(actionsRow());
    return;
  }

  const e = els[0];
  if (e.t === 'p') {
    h.textContent = 'Panel';
    const ids = new Set([...state.live.nodes.keys()]);
    if (e.n !== undefined) ids.add(e.n);
    const taken = new Map(state.layout.el.filter((x) => x.t === 'p' && x.n !== undefined && x !== e).map((x) => [x.n, true]));
    const options = [{ label: 'Not linked', value: '' }];
    for (const id of [...ids].sort((a, b) => a - b)) {
      const n = state.live.nodes.get(id);
      const parts = [`Module ${id}`];
      if (n && n.barcode) parts.push(n.barcode);
      if (taken.has(id)) parts.push('(on another panel)');
      options.push({ label: parts.join(' '), value: id });
    }
    pane.append(field('Module', selectEl(options, e.n ?? '', (v) => assignNode(e, v === '' ? undefined : Number(v)))));
    const orient = document.createElement('button');
    orient.className = 'btn';
    orient.type = 'button';
    orient.textContent = e.w <= e.h ? 'Switch to landscape' : 'Switch to portrait';
    orient.style.marginBottom = '0.9rem';
    orient.onclick = () => {
      checkpoint();
      const cx = e.x + e.w / 2;
      const cy = e.y + e.h / 2;
      [e.w, e.h] = [e.h, e.w];
      e.x = Math.round(cx - e.w / 2);
      e.y = Math.round(cy - e.h / 2);
      changed();
    };
    pane.append(field('String', stringSelect([e])));
    pane.append(orient, geometryFields(e));
  } else if (e.t === 'r') {
    h.textContent = 'Rectangle';
    pane.append(field('Label', input('textarea', e.l, (v) => { e.l = v || undefined; })));
    pane.append(valuePicker(e, 'l', true));
    const colors = document.createElement('div');
    colors.className = 'grid4';
    const fillL = document.createElement('label');
    fillL.append('Fill', input('color', e.f || '#6dad12', (v) => { e.f = v; }));
    const opL = document.createElement('label');
    opL.append('Fill opacity (%)', input('number', e.op ?? 12, (v) => { e.op = Math.max(0, Math.min(100, v)); }, { min: 0, max: 100, step: 5 }));
    const strokeL = document.createElement('label');
    strokeL.append('Border', input('color', e.s || '#78716c', (v) => { e.s = v; }));
    const swL = document.createElement('label');
    swL.append('Border width', input('number', e.sw ?? 2, (v) => { e.sw = Math.max(0, v); }, { min: 0, max: 20 }));
    const fsL = document.createElement('label');
    fsL.append('Label size', input('number', e.fs || 16, (v) => { e.fs = Math.max(6, v); }, { min: 6, max: 120 }));
    const cL = document.createElement('label');
    cL.append('Label color', input('color', e.c || '#d6d3d1', (v) => { e.c = v; }));
    colors.append(fillL, opL, strokeL, swL, fsL, cL);
    pane.append(colors, geometryFields(e));
  } else if (e.t === 'l') {
    h.textContent = 'Line';
    const grid = document.createElement('div');
    grid.className = 'grid4';
    const cL = document.createElement('label');
    cL.append('Color', input('color', e.s || '#a8a29e', (v) => { e.s = v; }));
    const wL = document.createElement('label');
    wL.append('Width', input('number', e.sw ?? 3, (v) => { e.sw = Math.max(1, v); }, { min: 1, max: 30 }));
    grid.append(cL, wL);
    const dash = document.createElement('label');
    dash.className = 'check';
    dash.append(input('checkbox', e.d, (v) => { e.d = v || undefined; }), 'Dashed');
    pane.append(grid, dash, geometryFields(e));
  } else if (e.t === 'x') {
    h.textContent = 'Text';
    pane.append(field('Text', input('textarea', e.txt, (v) => { e.txt = v; })));
    pane.append(valuePicker(e, 'txt', false));
    const grid = document.createElement('div');
    grid.className = 'grid4';
    const fsL = document.createElement('label');
    fsL.append('Size', input('number', e.fs || 18, (v) => { e.fs = Math.max(6, v); }, { min: 6, max: 200 }));
    const cL = document.createElement('label');
    cL.append('Color', input('color', e.c || '#fafaf9', (v) => { e.c = v; }));
    const aL = document.createElement('label');
    aL.append('Align', selectEl([
      { label: 'Left', value: 'start' }, { label: 'Center', value: 'middle' }, { label: 'Right', value: 'end' },
    ], e.a || 'start', (v) => { checkpoint(); e.a = v === 'start' ? undefined : v; changed(); }));
    grid.append(fsL, cL, aL);
    const bold = document.createElement('label');
    bold.className = 'check';
    bold.append(input('checkbox', e.b, (v) => { e.b = v || undefined; }), 'Bold');
    pane.append(grid, bold, geometryFields(e));
  }
  pane.append(actionsRow());
}

// ---------------------------------------------------------------------------
// Strings
// ---------------------------------------------------------------------------

function stringSelect(panels) {
  const first = panels[0].g;
  const same = panels.every((e) => e.g === first);
  const options = [{ label: 'None', value: '' }];
  if (!same) options.unshift({ label: 'Mixed', value: 'mixed' });
  for (const st of state.layout.st) options.push({ label: st.n, value: st.id });
  options.push({ label: 'New string…', value: 'new' });
  return selectEl(options, same ? (first ?? '') : 'mixed', (v) => {
    if (v === 'mixed') return;
    checkpoint();
    const id = v === 'new' ? createString().id : v === '' ? undefined : Number(v);
    for (const e of panels) {
      if (id === undefined) delete e.g;
      else e.g = id;
    }
    changed();
  });
}

// Adds a string without its own undo step (callers checkpoint).
function createString() {
  const id = state.layout.st.reduce((m, x) => Math.max(m, x.id), 0) + 1;
  const used = new Set(state.layout.st.map((x) => x.c));
  const c = STRING_COLORS.find((col) => !used.has(col)) || STRING_COLORS[(id - 1) % STRING_COLORS.length];
  const st = { id, n: `String ${id}`, c };
  state.layout.st.push(st);
  return st;
}

function deleteString(id) {
  checkpoint();
  state.layout.st = state.layout.st.filter((x) => x.id !== id);
  for (const e of state.layout.el) if (e.g === id) delete e.g;
  if (state.armedString === id) armString(null);
  changed();
}

function armString(id) {
  if (id !== null) state.armedNode = null;
  state.armedString = id;
  const st = id !== null ? stringById(id) : null;
  hint(st ? `Tap or swipe over panels to add them to ${st.n}. Tapping a member removes it. Esc ends.` : '');
  render();
  renderStringsPane();
  renderModules();
}

function renderStringsPane() {
  const pane = $('pane-strings');
  if (!pane || !state.editing) return;
  pane.replaceChildren();
  const intro = document.createElement('p');
  intro.className = 'muted';
  intro.style.marginBottom = '0.9rem';
  intro.textContent = 'Group panels into strings regardless of where they sit. Pick panels on the canvas, or select several and set the string under Properties.';
  pane.append(intro);

  for (const st of state.layout.st) {
    const box = document.createElement('div');
    box.className = 'string-edit' + (state.armedString === st.id ? ' armed' : '');
    box.style.color = st.c;
    const row = document.createElement('div');
    row.className = 'row';
    const name = input('text', st.n, (v) => { st.n = v || `String ${st.id}`; renderStringsCard(); });
    name.setAttribute('aria-label', 'String name');
    const color = input('color', st.c, (v) => { st.c = v; box.style.color = v; });
    color.setAttribute('aria-label', 'String color');
    row.append(name, color);
    const stats = stringStats(st.id);
    const info = document.createElement('div');
    info.className = 'stats';
    info.textContent = `${stats.count} panels, ${fmt(stats.power, 0)} W, ${fmt(stats.voltage, 1)} V`;
    const actions = document.createElement('div');
    actions.className = 'row';
    const pick = document.createElement('button');
    pick.type = 'button';
    pick.className = 'btn' + (state.armedString === st.id ? ' on' : '');
    pick.textContent = state.armedString === st.id ? 'Done' : 'Pick panels';
    pick.onclick = () => armString(state.armedString === st.id ? null : st.id);
    const del = document.createElement('button');
    del.type = 'button';
    del.className = 'btn danger';
    del.textContent = 'Delete';
    del.onclick = () => deleteString(st.id);
    actions.append(pick, del);
    actions.style.justifyContent = 'flex-start';
    box.append(row, info, actions);
    pane.append(box);
  }

  const add = document.createElement('button');
  add.type = 'button';
  add.className = 'btn primary';
  add.textContent = 'Add string';
  add.onclick = () => {
    checkpoint();
    const st = createString();
    changed();
    armString(st.id);
  };
  pane.append(add);
}

function renderStringsCard() {
  const card = $('strings-card');
  const show = state.layout.st.length > 0 && (!state.editing || state.sim.on);
  card.classList.toggle('show', show);
  card.classList.toggle('collapsed', state.stringsCollapsed);
  $('strings-toggle').setAttribute('aria-expanded', String(!state.stringsCollapsed));
  if (!show) return;
  const list = $('strings-list');
  list.replaceChildren();
  for (const st of state.layout.st) {
    const stats = stringStats(st.id);
    const li = document.createElement('li');
    const b = document.createElement('button');
    b.type = 'button';
    b.className = 'string-row' + (state.highlightString === st.id ? ' active' : '');
    const sw = document.createElement('span');
    sw.className = 'swatch';
    sw.style.background = st.c;
    const name = document.createElement('span');
    name.className = 'name';
    name.textContent = st.n;
    const pw = document.createElement('span');
    pw.className = 'pw';
    pw.textContent = `${fmt(stats.power, 0)} W`;
    const sub = document.createElement('span');
    sub.className = 'sub';
    sub.textContent = `${fmt(stats.voltage, 1)} V, ${stats.active} of ${stats.count} reporting`;
    b.append(sw, name, pw, sub);
    b.onclick = () => {
      if (state.editing) return;
      state.highlightString = state.highlightString === st.id ? null : st.id;
      render();
    };
    li.append(b);
    list.append(li);
  }
}

// ---------------------------------------------------------------------------
// Modules list (assignment)
// ---------------------------------------------------------------------------

function armNode(id) {
  if (id !== null && state.armedString !== null) {
    state.armedString = null;
    renderStringsPane();
  }
  state.armedNode = id;
  hint(id === null ? '' : `Tap the panel for module ${id}. Esc cancels.`);
  render();
  renderModules();
}

function renderModules() {
  const list = $('module-list');
  if (!list || !state.editing) return;
  const onPanel = new Map();
  for (const e of state.layout.el) if (e.t === 'p' && e.n !== undefined) onPanel.set(e.n, e);
  const ids = [...state.live.nodes.keys()].sort((a, b) => (onPanel.has(a) - onPanel.has(b)) || a - b);
  list.replaceChildren();
  if (!ids.length) {
    const li = document.createElement('li');
    li.className = 'muted';
    li.textContent = 'No modules reported yet. They show up as soon as the Tigo system sends data.';
    list.append(li);
    return;
  }
  for (const id of ids) {
    const n = state.live.nodes.get(id);
    const li = document.createElement('li');
    const b = document.createElement('button');
    b.type = 'button';
    b.className = 'module' + (onPanel.has(id) ? ' assigned' : '') + (state.armedNode === id ? ' armed' : '');
    const nid = document.createElement('span');
    nid.className = 'nid';
    nid.textContent = id;
    const bc = document.createElement('span');
    bc.className = 'bc';
    bc.textContent = n.barcode || 'Barcode not yet known';
    const w = document.createElement('span');
    w.className = 'w';
    w.textContent = n.fresh ? `${fmt(n.power, 0)} W` : '–';
    const st = document.createElement('span');
    st.className = 'st';
    st.textContent = onPanel.has(id) ? 'On a panel' : 'Not placed';
    b.append(nid, bc, w, st);
    b.onclick = () => {
      if (onPanel.has(id) && state.armedNode === null) {
        state.selected = new Set([onPanel.get(id)._id]);
        renderOverlay();
        switchPane('props');
        renderProps();
      } else {
        armNode(state.armedNode === id ? null : id);
      }
    };
    // Desktop: drag the module straight onto a panel.
    b.draggable = true;
    b.ondragstart = (ev) => ev.dataTransfer.setData('text/x-tigo-node', String(id));
    li.append(b);
    list.append(li);
  }
}

function switchPane(name) {
  document.querySelectorAll('.tabs button').forEach((b) => b.classList.toggle('active', b.dataset.pane === name));
  document.querySelectorAll('.pane').forEach((p) => p.classList.toggle('active', p.id === `pane-${name}`));
}

// ---------------------------------------------------------------------------
// Details card (view mode)
// ---------------------------------------------------------------------------

function renderDetails() {
  const card = $('details');
  const e = state.detailsPanel !== null ? byId(state.detailsPanel) : null;
  if (!e || state.editing) {
    card.classList.remove('show');
    return;
  }
  const linked = e.n !== undefined;
  const n = linked ? state.live.nodes.get(e.n) : null;
  const hasData = n && n.power !== undefined;
  const str = e.g !== undefined ? stringById(e.g) : null;

  $('d-title').textContent = linked ? `Module ${e.n}` : 'Panel';
  $('d-barcode').textContent = n && n.barcode ? n.barcode : '';

  const status = $('d-status');
  let label;
  let cls;
  if (!linked) { label = 'Not linked to a module'; cls = 'off'; }
  else if (!n) { label = 'Linked, module not seen yet'; cls = 'warn'; }
  else if (!hasData) { label = 'Linked, no readings yet'; cls = 'warn'; }
  else if (n.fresh) { label = `Live, updated ${n.age_s} s ago`; cls = 'ok'; }
  else { label = `No readings for ${fmtDuration(n.age_s)}`; cls = 'warn'; }
  status.textContent = label;
  status.className = `d-status ${cls}`;

  const strEl = $('d-string');
  strEl.replaceChildren();
  if (str) {
    const sw = document.createElement('span');
    sw.className = 'swatch';
    sw.style.background = str.c;
    strEl.append(sw, str.n);
  } else {
    strEl.append('No string');
  }

  const dl = $('d-list');
  dl.replaceChildren();
  if (hasData) {
    for (const key of ['power', 'voltage_in', 'voltage_out', 'current_in', 'temperature', 'duty_cycle', 'rssi']) {
      const m = METRICS[key];
      const dt = document.createElement('dt');
      dt.textContent = m.label;
      const dd = document.createElement('dd');
      dd.textContent = `${fmt(n[key], m.digits)}${m.unit ? ' ' + m.unit : ''}`;
      if (!n.fresh) dd.classList.add('old');
      dl.append(dt, dd);
    }
    if (n.gateway) {
      const dt = document.createElement('dt');
      dt.textContent = 'Via TAP';
      const dd = document.createElement('dd');
      dd.textContent = n.gateway;
      dl.append(dt, dd);
    }
  }
  const note = $('d-note');
  note.textContent = !linked
    ? 'Link this panel to its Tigo module to see live readings.'
    : hasData && !n.fresh ? 'Values are the last readings received.' : '';
  $('d-edit').hidden = linked;
  card.classList.add('show');
}

// ---------------------------------------------------------------------------
// Simulation (editor preview with made-up module data)
// ---------------------------------------------------------------------------

// Stable pseudo-random number in [0, 1) per key, so each panel keeps its
// character (orientation, shading) between updates.
function hash01(key) {
  let h = 2166136261;
  for (const ch of String(key)) h = Math.imul(h ^ ch.charCodeAt(0), 16777619);
  return ((h >>> 0) % 10000) / 10000;
}

function simulate() {
  const sun = state.sim.sun / 100;
  const nodes = new Map();
  let total = 0;
  let active = 0;
  for (const e of state.layout.el) {
    if (e.t !== 'p') continue;
    const key = e.n !== undefined ? e.n : `s${e._id}`;
    const h = hash01(key);
    // Most panels perform alike; about one in eight is partly shaded.
    const factor = h < 0.125 ? 0.35 + h * 2 : 0.88 + h * 0.12;
    const noise = 1 + (Math.random() - 0.5) * 0.04;
    const power = Math.max(0, 420 * sun * factor * noise);
    const vout = sun > 0.02 ? 31 + 6 * sun * factor + Math.random() * 0.3 : 0;
    const node = {
      id: key,
      barcode: '',
      power,
      voltage_out: vout,
      voltage_in: vout > 0 ? vout + 0.4 + Math.random() * 0.2 : 0,
      current_in: vout > 0 ? power / vout : 0,
      temperature: 12 + 38 * sun * factor + Math.random() * 0.5,
      rssi: -48 - Math.round(h * 30),
      duty_cycle: 255,
      age_s: 1,
      fresh: true,
    };
    nodes.set(key, node);
    total += power;
    active++;
  }
  state.sim.data = {
    nodes,
    system: { total_power: total, active_nodes: active, known_nodes: active, crc_errors: 0, uptime: 86400 },
  };
}

function setSimulation(on) {
  state.sim.on = on;
  clearInterval(state.sim.timer);
  state.sim.timer = null;
  if (on) {
    simulate();
    state.sim.timer = setInterval(() => { simulate(); render(); }, 2000);
  } else {
    state.sim.data = null;
  }
  document.body.classList.toggle('simulating', on);
  $('sim-btn').classList.toggle('sim-on', on);
  render();
}

// ---------------------------------------------------------------------------
// Modes, tools, load/save
// ---------------------------------------------------------------------------

function setTool(tool) {
  state.tool = tool;
  document.querySelectorAll('.tool').forEach((b) => b.classList.toggle('active', b.dataset.tool === tool));
  document.body.classList.toggle('tool-select', tool === 'select');
  document.body.classList.toggle('tool-place', tool !== 'select');
  const hints = {
    panel: 'Click to place a panel.',
    array: 'Click where the top-left panel should go.',
    rect: 'Drag to draw a rectangle.',
    line: 'Drag from start to end.',
    text: 'Click to place text.',
  };
  if (state.armedNode === null) hint(hints[tool] || '');
}

function setEditing(on) {
  if (!on && state.sim.on) setSimulation(false);
  state.editing = on;
  document.body.classList.toggle('editing', on);
  state.selected.clear();
  state.detailsPanel = null;
  state.highlightString = null;
  state.armedString = null;
  state.undo = [];
  state.redo = [];
  armNode(null);
  setTool('select');
  hint('');
  if (on) {
    renderProps();
    renderModules();
    renderStringsPane();
    updateToolbar();
    if (location.hash !== '#edit') history.replaceState(null, '', '#edit');
  } else if (location.hash) {
    history.replaceState(null, '', location.pathname);
  }
  render();
  renderDetails();
  // Sidebar/toolbar change the canvas size.
  requestAnimationFrame(fitView);
}

function isDirty() {
  return state.editing && layoutJson() !== state.saved;
}

async function loadLayout() {
  $('edit-btn').disabled = true;
  for (let attempt = 1; ; attempt++) {
    let res;
    try {
      res = await fetchWithTimeout('/api/layout', { cache: 'no-store' }, 8000);
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
    } catch (err) {
      // Never fall back to an empty layout here: saving it would wipe the
      // real one. Keep trying instead.
      console.error('Failed to load layout', err);
      hint(`Could not load the layout from the device. Retrying (attempt ${attempt})…`);
      await sleep(3000);
      continue;
    }
    const max = parseInt(res.headers.get('X-Layout-Max'));
    if (max >= 0) state.maxBytes = max;
    try {
      setLayout(JSON.parse(await res.text()));
    } catch (err) {
      setLayout({ el: [] });
      toast('The layout stored on the device is damaged. Import a backup or start a new one.', true);
    }
    state.saved = layoutJson();
    state.loaded = true;
    $('edit-btn').disabled = false;
    hint('');
    return;
  }
}

async function saveLayout() {
  const json = layoutJson();
  const bytes = new Blob([json]).size;
  if (bytes > state.maxBytes) {
    toast(`Layout is ${(bytes / 1024).toFixed(1)} KB, the device has room for ${(state.maxBytes / 1024).toFixed(1)} KB. Remove elements or shorten labels.`, true);
    return;
  }
  const btn = $('save-btn');
  btn.disabled = true;
  btn.textContent = 'Saving…';
  try {
    const res = await fetchWithTimeout('/api/layout', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: json }, 20000);
    const max = parseInt(res.headers.get('X-Layout-Max'));
    if (max >= 0) state.maxBytes = max;
    if (!res.ok) throw new Error(await res.text());
    // Read it back, so "saved" means it really is on the device.
    const check = await fetchWithTimeout('/api/layout', { cache: 'no-store' }, 8000);
    if (!check.ok || (await check.text()) !== json) throw new Error('the device returned a different layout when checked');
    state.saved = json;
    toast('Layout saved');
    setEditing(false);
  } catch (err) {
    const reason = err.name === 'AbortError' ? 'the device did not answer in time' : err.message || 'device not reachable';
    toast(`Layout not saved: ${reason}. Your changes are still here; use Export to keep a copy.`, true);
  } finally {
    btn.disabled = false;
    btn.textContent = 'Save layout';
  }
}

function exportLayout() {
  const blob = new Blob([JSON.stringify(JSON.parse(layoutJson()), null, 1)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'tigotell-layout.json';
  document.body.append(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

function importLayout(file) {
  const reader = new FileReader();
  reader.onload = () => {
    try {
      const obj = JSON.parse(reader.result);
      if (!Array.isArray(obj.el)) throw new Error('no elements');
      checkpoint();
      setLayout(obj);
      changed();
      fitView();
      toast('Layout imported. Save to keep it on the device.');
    } catch (err) {
      toast('This file is not a TigoTell layout.', true);
    }
  };
  reader.readAsText(file);
}

// ---------------------------------------------------------------------------
// Wiring
// ---------------------------------------------------------------------------

function init() {
  const canvas = $('canvas');
  canvas.addEventListener('pointerdown', onPointerDown);
  canvas.addEventListener('pointermove', onPointerMove);
  canvas.addEventListener('pointerup', onPointerUp);
  canvas.addEventListener('pointercancel', onPointerUp);
  canvas.addEventListener('wheel', (ev) => {
    ev.preventDefault();
    zoomAt(Math.exp(-ev.deltaY * (ev.deltaMode === 1 ? 0.05 : 0.0015)), ev.clientX, ev.clientY);
  }, { passive: false });
  canvas.addEventListener('dragover', (ev) => {
    if (state.editing && ev.dataTransfer.types.includes('text/x-tigo-node')) ev.preventDefault();
  });
  canvas.addEventListener('drop', (ev) => {
    const id = Number(ev.dataTransfer.getData('text/x-tigo-node'));
    const node = document.elementFromPoint(ev.clientX, ev.clientY);
    const g = node && node.closest('[data-id]');
    const e = g && byId(Number(g.getAttribute('data-id')));
    ev.preventDefault();
    if (e && e.t === 'p') {
      assignNode(e, id);
      toast(`Module ${id} linked to panel`);
    } else {
      toast('Drop the module onto a panel.', true);
    }
  });

  $('zoom-in').onclick = () => zoomCenter(1.25);
  $('zoom-out').onclick = () => zoomCenter(0.8);
  $('zoom-fit').onclick = fitView;
  $('edit-btn').onclick = () => setEditing(true);
  $('empty-edit').onclick = () => setEditing(true);
  $('cancel-btn').onclick = () => {
    if (isDirty() && !confirm('Discard your changes to the layout?')) return;
    setLayout(JSON.parse(state.saved));
    setEditing(false);
  };
  $('save-btn').onclick = saveLayout;
  $('undo-btn').onclick = undo;
  $('redo-btn').onclick = redo;
  $('snap-btn').onclick = () => { state.snap = !state.snap; updateToolbar(); };
  $('export-btn').onclick = exportLayout;
  $('sim-btn').onclick = () => setSimulation(!state.sim.on);
  const sun = $('sim-sun');
  sun.oninput = () => {
    state.sim.sun = Number(sun.value);
    $('sim-sun-value').textContent = `${sun.value} %`;
    if (state.sim.on) { simulate(); render(); }
  };
  const simMetric = $('sim-metric');
  for (const [k, m] of Object.entries(METRICS)) {
    if (k !== 'duty_cycle') simMetric.append(new Option(m.label, k));
  }
  simMetric.append(new Option('String', 'string'));
  $('import-btn').onclick = () => $('import-file').click();
  $('import-file').onchange = (ev) => {
    if (ev.target.files[0]) importLayout(ev.target.files[0]);
    ev.target.value = '';
  };
  $('strings-toggle').onclick = () => {
    state.stringsCollapsed = !state.stringsCollapsed;
    renderStringsCard();
  };
  $('d-close').onclick = () => { state.detailsPanel = null; renderDetails(); renderOverlay(); };
  $('d-edit').onclick = () => {
    const id = state.detailsPanel;
    setEditing(true);
    if (byId(id)) {
      state.selected = new Set([id]);
      switchPane('props');
      renderOverlay();
      renderProps();
    }
  };
  document.querySelectorAll('.tool').forEach((b) => (b.onclick = () => { armString(null); armNode(null); setTool(b.dataset.tool); }));
  document.querySelectorAll('.tabs button').forEach((b) => (b.onclick = () => switchPane(b.dataset.pane)));

  const metricSel = $('metric');
  try { state.metric = localStorage.getItem('tigotell.metric') || 'power'; } catch (e) { /* storage unavailable */ }
  if (!METRICS[state.metric] && state.metric !== 'string') state.metric = 'power';
  metricSel.value = state.metric;
  simMetric.value = state.metric;
  simMetric.onchange = () => {
    state.metric = metricSel.value = simMetric.value;
    try { localStorage.setItem('tigotell.metric', state.metric); } catch (e) { /* ignore */ }
    render();
  };
  metricSel.onchange = () => {
    simMetric.value = metricSel.value;
    state.metric = metricSel.value;
    try { localStorage.setItem('tigotell.metric', state.metric); } catch (e) { /* ignore */ }
    render();
  };

  document.addEventListener('keydown', (ev) => {
    if (!state.editing) return;
    const tag = ev.target.tagName;
    if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;
    const mod = ev.ctrlKey || ev.metaKey;
    const key = ev.key.toLowerCase();
    if (mod && key === 'z') { ev.preventDefault(); ev.shiftKey ? redo() : undo(); }
    else if (mod && key === 'y') { ev.preventDefault(); redo(); }
    else if (mod && key === 'd') { ev.preventDefault(); duplicateSelected(); }
    else if (mod && key === 'a') { ev.preventDefault(); state.selected = new Set(state.layout.el.map((e) => e._id)); renderOverlay(); renderProps(); }
    else if (key === 'delete' || key === 'backspace') { ev.preventDefault(); deleteSelected(); }
    else if (key === 'escape') { armString(null); armNode(null); setTool('select'); state.selected.clear(); renderOverlay(); renderProps(); }
    else if (key.startsWith('arrow') && state.selected.size) {
      ev.preventDefault();
      const step = ev.altKey ? 1 : GRID;
      checkpoint();
      moveSelected(key === 'arrowleft' ? -step : key === 'arrowright' ? step : 0, key === 'arrowup' ? -step : key === 'arrowdown' ? step : 0);
      changed();
    } else if (!mod) {
      const tools = { v: 'select', p: 'panel', a: 'array', r: 'rect', l: 'line', t: 'text' };
      if (tools[key]) { armNode(null); setTool(tools[key]); }
    }
  });

  window.addEventListener('beforeunload', (ev) => {
    if (isDirty()) { ev.preventDefault(); ev.returnValue = ''; }
  });
  window.addEventListener('resize', () => renderOverlay());
  window.addEventListener('hashchange', () => {
    if (location.hash === '#edit' && !state.editing && state.loaded) setEditing(true);
  });
}

async function start() {
  init();
  applyView();
  render();
  await loadLayout();
  await fetchLive();
  if (location.hash === '#edit') setEditing(true);
  else { render(); requestAnimationFrame(fitView); }
  setTimeout(pollLive, 5000);
}

start();
