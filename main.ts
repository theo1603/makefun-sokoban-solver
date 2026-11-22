// A push-optimal Sokoban solver for make.fun's sokoban-solver-01.
//
// The generated levels are a one-wide hallway with independent rooms off it.
// A box pushed into the hallway can never reach a goal again, so each room is
// a problem of its own and the level's push optimum is the sum of its rooms'.
// This program splits the level into rooms and solves each room push-optimally,
// walking the player on between rooms.
//
// Each room is solved by A* over pushes. A state is the boxes' squares plus the
// player's reachable region (named by its lowest square), so walking never
// branches the search. The heuristic is the cheapest assignment of boxes to
// goals under per-goal push distances (a reverse BFS that ignores the other
// boxes), which never overestimates, so the weight-1 search is push-optimal.
// Dead squares (no push path to any goal) and frozen 2x2 blocks prune hopeless
// states. Everything is budgeted by expansions, never by the clock, and every
// tie-break is fixed, so two runs write identical bytes. If the optimal search
// runs out of budget on a room, weighted A* and then greedy best-first fall
// back to a merely solving answer (partial points) rather than none.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const INPUT_DIR = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const OUTPUT_DIR = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Work budgets, in state expansions: work, never the clock, so two runs agree.
const OPTIMAL_EXPANSIONS = 300_000; // per room, push-optimal A*
const WEIGHTED_EXPANSIONS = 150_000; // per room, weighted A* fallback
const GREEDY_EXPANSIONS = 100_000; // per room, greedy fallback
const MAX_STATES = 600_000; // states kept per search (a memory bound)
const GLOBAL_EXPANSIONS = 2_000_000; // whole set; past this, searches give up fast
const meter = { expansions: 0 };

const DIRS = [
  [0, -1, 'u'],
  [0, 1, 'd'],
  [-1, 0, 'l'],
  [1, 0, 'r'],
];

/** XSB text (# wall, space floor, . goal, $ box, * box on goal, @ player) to a board. */
function parseLevel(text) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals = [];
  const boxes = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  // The outer frame is wall whatever it shows: nothing leaves the board.
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
  boxes.sort((a, b) => a - b);
  goals.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

/**
 * For each goal, the fewest pushes that bring a box from any square onto that
 * goal, ignoring the other boxes: a reverse BFS from the goal, where a box at s
 * could have been pushed from s-d when the player could stand at s-2d. A square
 * is live when some goal's table reaches it.
 */
function pushDistances(w, wall, goals) {
  const size = wall.length;
  const tables = [];
  const live = new Uint8Array(size);
  for (const g of goals) {
    const dist = new Int16Array(size).fill(-1);
    dist[g] = 0;
    live[g] = 1;
    const queue = [g];
    for (let qi = 0; qi < queue.length; qi++) {
      const s = queue[qi];
      for (const [dx, dy] of DIRS) {
        const d = dy * w + dx;
        const from = s - d;
        if (wall[from] || wall[from - d] || dist[from] >= 0) continue;
        dist[from] = dist[s] + 1;
        live[from] = 1;
        queue.push(from);
      }
    }
    tables.push(dist);
  }
  return { tables, live };
}

/**
 * The level's rooms: cut the dead squares of the player's row (the hallway: a
 * box there reaches no goal) out of the floor; each part left holding boxes is
 * a room. Null when some part holds more or fewer boxes than goals, a goal
 * stands on a cut square, or a box starts in the hallway: then the level is no
 * set of rooms.
 */
function findRooms(level, live) {
  const { w, wall, goals, boxes, player } = level;
  const size = wall.length;
  const row = Math.floor(player / w);
  const cut = new Uint8Array(size);
  for (let i = row * w; i < (row + 1) * w; i++) if (!wall[i] && !live[i]) cut[i] = 1;
  if (boxes.some((b) => cut[b])) return null;
  const comp = new Int32Array(size).fill(-1);
  const rooms = [];
  for (let s = 0; s < size; s++) {
    if (wall[s] || cut[s] || comp[s] >= 0) continue;
    const queue = [s];
    comp[s] = rooms.length;
    for (let i = 0; i < queue.length; i++) {
      const p = queue[i];
      for (const [dx, dy] of DIRS) {
        const q = p + dy * w + dx;
        if (!wall[q] && !cut[q] && comp[q] < 0) {
          comp[q] = rooms.length;
          queue.push(q);
        }
      }
    }
    rooms.push({ boxes: [], goals: [] });
  }
  for (const b of boxes) rooms[comp[b]].boxes.push(b);
  for (const g of goals) {
    if (comp[g] < 0) return null;
    rooms[comp[g]].goals.push(g);
  }
  const withBoxes = rooms.filter((r) => r.boxes.length > 0);
  if (withBoxes.length === 0) return null;
  if (withBoxes.some((r) => r.boxes.length !== r.goals.length)) return null;
  if (rooms.some((r) => r.boxes.length === 0 && r.goals.length > 0)) return null;
  for (const r of withBoxes) {
    r.boxes.sort((a, b) => a - b);
    r.goals.sort((a, b) => a - b);
  }
  return withBoxes;
}

/** The squares the player reaches from `from` around walls and boxes, marked in `seen` with `stamp`; the lowest. */
function flood(w, wall, occupied, from, seen, stamp, stack) {
  let low = from;
  let n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n];
    if (p < low) low = p;
    const a = p - w;
    if (wall[a] === 0 && occupied[a] === 0 && seen[a] !== stamp) { seen[a] = stamp; stack[n++] = a; }
    const b = p + w;
    if (wall[b] === 0 && occupied[b] === 0 && seen[b] !== stamp) { seen[b] = stamp; stack[n++] = b; }
    const c = p - 1;
    if (wall[c] === 0 && occupied[c] === 0 && seen[c] !== stamp) { seen[c] = stamp; stack[n++] = c; }
    const d = p + 1;
    if (wall[d] === 0 && occupied[d] === 0 && seen[d] !== stamp) { seen[d] = stamp; stack[n++] = d; }
  }
  return low;
}

/** Whether the box just pushed to `t` sits in a frozen 2x2 block of walls and boxes with a box off its goal. */
function frozenBlock(w, wall, occupied, goal, t) {
  for (const a of [t, t - 1, t - w, t - w - 1]) {
    const b = a + 1;
    const c = a + w;
    const d = c + 1;
    if (!(wall[a] || occupied[a]) || !(wall[b] || occupied[b]) || !(wall[c] || occupied[c]) || !(wall[d] || occupied[d])) continue;
    if ((occupied[a] && !goal[a]) || (occupied[b] && !goal[b]) || (occupied[c] && !goal[c]) || (occupied[d] && !goal[d])) return true;
  }
  return false;
}

/** All permutations of 0..n-1, for the assignment heuristic. */
function permutations(n) {
  const out = [];
  const arr = Array.from({ length: n }, (_, i) => i);
  const gen = (k) => {
    if (k === n) { out.push(arr.slice()); return; }
    for (let i = k; i < n; i++) {
      const t = arr[k]; arr[k] = arr[i]; arr[i] = t;
      gen(k + 1);
      arr[i] = arr[k]; arr[k] = t;
    }
  };
  gen(0);
  return out;
}

/** The shortest walk from `from` to `to` around walls and boxes, in lower LURD letters (u first on ties). */
function walkTo(w, wall, occupied, from, to) {
  if (from === to) return '';
  const prev = new Int32Array(wall.length).fill(-1);
  const how = new Array(wall.length);
  const queue = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi];
    if (p === to) break;
    for (const [dx, dy, letter] of DIRS) {
      const q = p + dy * w + dx;
      if (wall[q] || occupied[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = letter;
      queue.push(q);
    }
  }
  if (prev[to] === -1) throw new Error('walkTo: the push square is out of reach');
  let s = '';
  for (let p = to; p !== from; p = prev[p]) s = how[p] + s;
  return s;
}

/**
 * A* over pushes for one board: {pushes, moves} or null when a budget runs out
 * (or the board has no solution). weight 1 with the assignment heuristic is
 * push-optimal; a higher weight, or greedy, trades optimality for speed.
 */
function search(board, opts) {
  const { w, wall, goals, boxes: startBoxes, player: startPlayer } = board;
  const { weight, greedy, maxExpansions, maxStates } = opts;
  const size = wall.length;
  const n = startBoxes.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const { tables, live } = pushDistances(w, wall, goals);
  const solved = (bs) => bs.every((b) => goal[b]);
  if (solved(startBoxes)) return { pushes: 0, moves: '' };
  if (startBoxes.some((b) => !live[b])) return null;

  // The heuristic: the cheapest assignment of boxes to goals under the push
  // distances (exact for few boxes; a per-box nearest goal when there are many).
  const useAssign = n <= 5;
  const perms = useAssign ? permutations(n) : null;
  const heuristic = (bs) => {
    if (useAssign) {
      let best = Infinity;
      for (const p of perms) {
        let s = 0;
        for (let i = 0; i < n; i++) {
          const d = tables[p[i]][bs[i]];
          if (d < 0) { s = Infinity; break; }
          s += d;
        }
        if (s < best) best = s;
      }
      return best;
    }
    let s = 0;
    for (const b of bs) {
      let m = Infinity;
      for (const t of tables) {
        const d = t[b];
        if (d >= 0 && d < m) m = d;
      }
      if (m === Infinity) return Infinity;
      s += m;
    }
    return s;
  };

  // A state's key: the region's lowest square and the boxes' squares, as digits
  // in base `size` while that stays exact, else as a short string.
  const numericKey = size ** (n + 1) < 2 ** 53;
  const keyOf = numericKey
    ? (bs, low) => { let k = low; for (const b of bs) k = k * size + b; return k; }
    : (bs, low) => String.fromCharCode(low, ...bs);

  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;

  const sBoxes = [startBoxes.slice()];
  const sPlayer = [startPlayer];
  const sParent = [-1];
  const sG = [0];
  const sKey = [];
  for (const b of startBoxes) occupied[b] = 1;
  sKey.push(keyOf(startBoxes, flood(w, wall, occupied, startPlayer, seen, ++stamp, stack)));
  for (const b of startBoxes) occupied[b] = 0;
  const best = new Map([[sKey[0], 0]]);

  const h0 = heuristic(startBoxes);
  if (h0 === Infinity) return null;

  // The open list: a binary heap of [f, h, seq, state], smallest first; seq is
  // the insertion order, so ties break the same on every run.
  const heap = [];
  let seq = 0;
  const fOf = (g, h) => (greedy ? h : g + weight * h);
  const less = (a, b) => a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e) => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i], heap[p])) break;
      const t = heap[i]; heap[i] = heap[p]; heap[p] = t;
      i = p;
    }
  };
  const heapPop = () => {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1], heap[c])) c++;
        if (!less(heap[c], heap[i])) break;
        const t = heap[i]; heap[i] = heap[c]; heap[c] = t;
        i = c;
      }
    }
    return top;
  };
  heapPush([fOf(0, h0), h0, seq++, 0]);

  let expansions = 0;
  let found = -1;
  while (heap.length > 0) {
    const entry = heapPop();
    const idx = entry[3];
    if (best.get(sKey[idx]) !== sG[idx]) continue; // stale: a better route to this state exists
    const boxes = sBoxes[idx];
    if (solved(boxes)) { found = idx; break; }
    if (expansions >= maxExpansions || meter.expansions >= GLOBAL_EXPANSIONS) return null;
    expansions++;
    meter.expansions++;
    const g = sG[idx];
    for (const b of boxes) occupied[b] = 1;
    const region = ++stamp;
    flood(w, wall, occupied, sPlayer[idx], seen, region, stack);
    for (let bi = 0; bi < n; bi++) {
      const b = boxes[bi];
      for (let k = 0; k < 4; k++) {
        const d = DIRS[k][1] * w + DIRS[k][0];
        const t = b + d;
        if (wall[t] || occupied[t] || !live[t]) continue;
        if (seen[b - d] !== region) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        const frozen = frozenBlock(w, wall, occupied, goal, t);
        let key = null;
        let next = null;
        if (!frozen) {
          next = boxes.slice();
          next[bi] = t;
          next.sort((x, y) => x - y);
          key = keyOf(next, flood(w, wall, occupied, b, mark, ++stamp, stack));
        }
        occupied[t] = 0;
        occupied[b] = 1;
        if (frozen || key === null) continue;
        const g2 = g + 1;
        const prev = best.get(key);
        if (prev !== undefined && prev <= g2) continue;
        const h2 = heuristic(next);
        if (h2 === Infinity) continue;
        if (sBoxes.length >= maxStates) return null;
        best.set(key, g2);
        const ni = sBoxes.length;
        sBoxes.push(next);
        sPlayer.push(b);
        sParent.push(idx);
        sG.push(g2);
        sKey.push(key);
        heapPush([fOf(g2, h2), h2, seq++, ni]);
      }
    }
    for (const b of boxes) occupied[b] = 0;
  }
  if (found < 0) return null;
  // The path of pushes, then the moves: walk to each push's square, then push.
  const path = [];
  for (let s = found; s > 0; s = sParent[s]) path.push(s);
  path.reverse();
  let moves = '';
  let player = startPlayer;
  let boxes = startBoxes;
  for (const s of path) {
    const next = sBoxes[s];
    const from = boxes.find((b) => !next.includes(b));
    const to = next.find((b) => !boxes.includes(b));
    const d = to - from;
    for (const b of boxes) occupied[b] = 1;
    moves += walkTo(w, wall, occupied, player, from - d);
    for (const b of boxes) occupied[b] = 0;
    moves += DIRS.find(([dx, dy]) => dy * w + dx === d)[2];
    player = from;
    boxes = next;
  }
  return { pushes: path.length, moves };
}

/** The square the player stands on after `moves` from `from` on a board `w` wide. */
function endOf(w, from, moves) {
  let p = from;
  for (const ch of moves) p += ch === 'u' ? -w : ch === 'd' ? w : ch === 'l' ? -1 : 1;
  return p;
}

/** One room solved, optimal first, then weighted, then greedy. */
function solveRoom(sub) {
  let res = search(sub, { weight: 1, greedy: false, maxExpansions: OPTIMAL_EXPANSIONS, maxStates: MAX_STATES });
  if (res === null) res = search(sub, { weight: 3, greedy: false, maxExpansions: WEIGHTED_EXPANSIONS, maxStates: MAX_STATES });
  if (res === null) res = search(sub, { weight: 1, greedy: true, maxExpansions: GREEDY_EXPANSIONS, maxStates: MAX_STATES });
  return res;
}

/** The moves solving one level in XSB, or '' when no search in budget solves it. */
function solveLevel(text) {
  const level = parseLevel(text);
  const { w, wall, goals, boxes, player } = level;
  const goalSet = new Set(goals);
  if (boxes.every((b) => goalSet.has(b))) return { moves: '', pushes: 0 };
  const { live } = pushDistances(w, wall, goals);
  const rooms = findRooms(level, live);
  if (rooms !== null) {
    // Each room's boxes as they stand; a solved room's sit on its goals.
    const current = rooms.map((r) => r.boxes.slice());
    let playerNow = player;
    let moves = '';
    let pushes = 0;
    for (let ri = 0; ri < rooms.length; ri++) {
      const room = rooms[ri];
      const subWall = wall.slice();
      for (let rj = 0; rj < rooms.length; rj++) {
        if (rj === ri) continue;
        for (const b of current[rj]) subWall[b] = 1;
      }
      const res = solveRoom({ w, wall: subWall, goals: room.goals, boxes: current[ri], player: playerNow });
      if (res === null) return { moves: '', pushes: 0, failed: true };
      moves += res.moves;
      pushes += res.pushes;
      current[ri] = room.goals.slice();
      playerNow = endOf(w, playerNow, res.moves);
    }
    return { moves, pushes };
  }
  // No room split: search the whole level, optimal first, then the fallbacks.
  const res = solveRoom({ w, wall, goals, boxes, player });
  if (res === null) return { moves: '', pushes: 0, failed: true };
  return { moves: res.moves, pushes: res.pushes };
}

for (const id of readdirSync(INPUT_DIR).sort()) {
  let moves = '';
  let info = '';
  try {
    const text = readFileSync(join(INPUT_DIR, id, 'level.txt'), 'utf8');
    const before = meter.expansions;
    const res = solveLevel(text);
    // An answer over the checker's size cap would not count at all; an empty
    // one at least keeps the instance answered in its stated format.
    moves = res.moves.length + 1 <= 65536 ? res.moves : '';
    info = `pushes=${res.pushes} expansions=${meter.expansions - before}${res.failed ? ' FAILED' : ''}`;
  } catch (err) {
    info = `error: ${err.message}`;
  }
  mkdirSync(join(OUTPUT_DIR, id), { recursive: true });
  writeFileSync(join(OUTPUT_DIR, id, 'solution.txt'), `${moves}\n`);
  console.error(`${id}: ${info}`);
}
