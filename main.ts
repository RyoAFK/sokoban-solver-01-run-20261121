// sokoban-solver: a push-optimal Sokoban solver for make.fun's generated levels.
//
// The generator's levels are a one-wide hallway with independent rooms off it,
// and a level's push-optimal count is the sum of its rooms' optima. This solver
// exploits exactly that: it splits the level into the rooms the hallway row
// separates, solves each room push-optimally with A* over pushes, and joins the
// room solutions with walking paths. If a level does not split cleanly it falls
// back to a whole-level push A*.
//
// The room search is a push-based A* (state: the room's boxes plus the player's
// reachable region, canonicalised to its smallest square) with:
//   - an admissible heuristic: the min-cost assignment of boxes to goals over
//     precomputed single-box push distances (Hungarian algorithm),
//   - dead-square pruning: squares from which no goal is reachable by pushes,
//   - 2x2 freeze-deadlock pruning.
// All budgets are counted in expansions, never in wall time, so two runs of the
// program on the same set produce byte-identical answers.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Directions in a fixed order: up, down, left, right.
const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const CH = ['u', 'd', 'l', 'r'];
const CH_UP = ['U', 'D', 'L', 'R'];

const ROOM_EXPANSION_CAP = 1500000;
const LEVEL_EXPANSION_CAP = 6000000;
const WHOLE_EXPANSION_CAP = 4000000;
const BIG = 1e9;

function solveLevel(text, budget) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const N = w * h;
  const wall = new Uint8Array(N);
  const isGoal = new Uint8Array(N);
  const goalsAll = [];
  let boxesAll = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < w; x++) {
      const ch = row[x] ?? '#';
      const i = y * w + x;
      if (ch === '#') wall[i] = 1;
      if ('.*+'.includes(ch)) { isGoal[i] = 1; goalsAll.push(i); }
      if ('$*'.includes(ch)) boxesAll.push(i);
      if ('@+'.includes(ch)) player = i;
    }
  });
  boxesAll.sort((a, b) => a - b);
  if (player < 0 || boxesAll.length === 0) return '';
  const hallwayY = Math.floor(player / w);

  // ---- shared helpers -------------------------------------------------------

  // Flood fill of the player's reachable region avoiding `occ` boxes.
  const vis = new Int32Array(N);
  let visStamp = 0;
  function flood(start, occ) {
    visStamp++;
    const stack = [start];
    vis[start] = visStamp;
    let min = start;
    while (stack.length > 0) {
      const c = stack.pop();
      if (c < min) min = c;
      const cx = c % w, cy = Math.floor(c / w);
      for (let d = 0; d < 4; d++) {
        const nx = cx + DX[d], ny = cy + DY[d];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const n = ny * w + nx;
        if (wall[n] || occ[n] || vis[n] === visStamp) continue;
        vis[n] = visStamp;
        stack.push(n);
      }
    }
    return min;
  }
  const inRegion = (c) => vis[c] === visStamp;

  // BFS walking path from `from` to any cell of `targets` (a Set) avoiding occ.
  // Returns the move string, or null when unreachable.
  function walkPath(from, targets, occ) {
    if (targets.has(from)) return '';
    const prev = new Int32Array(N).fill(-2);
    const prevDir = new Int32Array(N);
    const queue = [from];
    prev[from] = -1;
    let head = 0;
    while (head < queue.length) {
      const c = queue[head++];
      const cx = c % w, cy = Math.floor(c / w);
      for (let d = 0; d < 4; d++) {
        const nx = cx + DX[d], ny = cy + DY[d];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const n = ny * w + nx;
        if (wall[n] || occ[n] || prev[n] !== -2) continue;
        prev[n] = c;
        prevDir[n] = d;
        if (targets.has(n)) {
          let s = '';
          let cur = n;
          while (prev[cur] !== -1) { s = CH[prevDir[cur]] + s; cur = prev[cur]; }
          return s;
        }
        queue.push(n);
      }
    }
    return null;
  }

  // Single-box push distances to each goal, restricted to box cells `inS`:
  // dist[g][c] = min pushes to bring a box from c to goal g ignoring other
  // boxes (the player may stand on any floor square). Also returns dead[]:
  // squares of S that reach no goal.
  function pullDistances(S, goals) {
    const dists = goals.map((g) => {
      const dist = new Float64Array(N).fill(Infinity);
      dist[g] = 0;
      const queue = [g];
      let head = 0;
      while (head < queue.length) {
        const c = queue[head++];
        const cx = c % w, cy = Math.floor(c / w);
        for (let d = 0; d < 4; d++) {
          // A push in direction d moved a box from p = c - d into c, with the
          // player standing at p - d = c - 2d.
          const px = cx - DX[d], py = cy - DY[d];
          const sx = cx - 2 * DX[d], sy = cy - 2 * DY[d];
          if (px < 0 || py < 0 || px >= w || py >= h || sx < 0 || sy < 0 || sx >= w || sy >= h) continue;
          const p = py * w + px, s = sy * w + sx;
          if (!S.has(p) || wall[s] || dist[p] !== Infinity) continue;
          dist[p] = dist[c] + 1;
          queue.push(p);
        }
      }
      return dist;
    });
    const dead = new Uint8Array(N);
    for (const c of S) {
      if (isGoal[c]) continue;
      let ok = false;
      for (const dist of dists) if (dist[c] !== Infinity) { ok = true; break; }
      if (!ok) dead[c] = 1;
    }
    return { dists, dead };
  }

  // Hungarian algorithm (rectangular, rows <= cols): min cost assignment.
  function hungarian(cost, n, m) {
    const u = new Float64Array(n + 1);
    const v = new Float64Array(m + 1);
    const p = new Int32Array(m + 1);
    const way = new Int32Array(m + 1);
    for (let i = 1; i <= n; i++) {
      p[0] = i;
      let j0 = 0;
      const minv = new Float64Array(m + 1).fill(Infinity);
      const used = new Uint8Array(m + 1);
      do {
        used[j0] = 1;
        const i0 = p[j0];
        let delta = Infinity, j1 = -1;
        for (let j = 1; j <= m; j++) {
          if (used[j]) continue;
          const cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
          if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
          if (minv[j] < delta) { delta = minv[j]; j1 = j; }
        }
        for (let j = 0; j <= m; j++) {
          if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
          else minv[j] -= delta;
        }
        j0 = j1;
      } while (p[j0] !== 0);
      do {
        const j1 = way[j0];
        p[j0] = p[j1];
        j0 = j1;
      } while (j0 !== 0);
    }
    let total = 0;
    for (let j = 1; j <= m; j++) {
      if (p[j] > 0) total += cost[p[j] - 1][j - 1];
    }
    return total;
  }

  // ---- the push A* ----------------------------------------------------------
  // Solves one independent subproblem: boxes `boxes0` must all reach `goals`,
  // boxes may only stand on cells of S, `fixedOcc` marks other boxes that act
  // as static obstacles, the player starts at `playerStart`. When needExit is
  // set, the final position must let the player walk back to the hallway row.
  // Returns { pushes: [{from,to,dir}], playerEnd } or null.
  function pushSearch(S, boxes0, goals, fixedOcc, playerStart, needExit, cap) {
    const { dists, dead } = pullDistances(S, goals);
    const nb = boxes0.length;
    // Cost matrix builder for a box set: rows = boxes, cols = goals.
    const hCache = new Map();
    function heuristic(boxes) {
      const key = String.fromCharCode(...boxes);
      const hit = hCache.get(key);
      if (hit !== undefined) return hit;
      const cost = boxes.map((b) => dists.map((dist) => (dist[b] === Infinity ? BIG : dist[b])));
      // Pad to square if needed (boxes == goals in every use, but stay safe).
      let val;
      if (nb === goals.length) val = hungarian(cost, nb, goals.length);
      else {
        val = 0;
        for (const row of cost) val += Math.min(...row);
      }
      if (val >= BIG) val = Infinity;
      hCache.set(key, val);
      return val;
    }
    for (const b of boxes0) if (dead[b]) return null;
    const h0 = heuristic(boxes0);
    if (h0 === Infinity) return null;

    const occ = new Uint8Array(N);
    for (let i = 0; i < N; i++) if (fixedOcc[i]) occ[i] = 1;

    // Node storage.
    const nBoxes = [boxes0.slice()];
    const nPlayer = [playerStart];
    const nG = [0];
    const nParent = [-1];
    const nFrom = [-1];
    const nTo = [-1];
    const nDir = [-1];
    const nKey = [''];
    const best = new Map();
    // Heap of [f, h, counter, nodeIndex].
    const heap = [];
    let counter = 0;
    const less = (a, b) => a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
    function heapPush(e) {
      heap.push(e);
      let i = heap.length - 1;
      while (i > 0) {
        const p = (i - 1) >> 1;
        if (!less(heap[i], heap[p])) break;
        [heap[i], heap[p]] = [heap[p], heap[i]];
        i = p;
      }
    }
    function heapPop() {
      const top = heap[0];
      const last = heap.pop();
      if (heap.length > 0) {
        heap[0] = last;
        for (let i = 0; ; ) {
          let c = 2 * i + 1;
          if (c >= heap.length) break;
          if (c + 1 < heap.length && less(heap[c + 1], heap[c])) c++;
          if (!less(heap[c], heap[i])) break;
          [heap[i], heap[c]] = [heap[c], heap[i]];
          i = c;
        }
      }
      return top;
    }
    function stateKey(canonPlayer, boxes) {
      return String.fromCharCode(canonPlayer, ...boxes);
    }
    // 2x2 freeze check around the landing cell of a push.
    function freezeDead(land) {
      const lx = land % w, ly = Math.floor(land / w);
      for (let ox = -1; ox <= 0; ox++) {
        for (let oy = -1; oy <= 0; oy++) {
          let allBlocked = true;
          let badBox = false;
          for (let dx = 0; dx <= 1 && allBlocked; dx++) {
            for (let dy = 0; dy <= 1; dy++) {
              const x = lx + ox + dx, y = ly + oy + dy;
              if (x < 0 || y < 0 || x >= w || y >= h) { /* outside counts as wall */ }
              else {
                const c = y * w + x;
                if (wall[c] || occ[c]) { if (occ[c] && !isGoal[c]) badBox = true; }
                else allBlocked = false;
              }
            }
          }
          if (allBlocked && badBox) return true;
        }
      }
      return false;
    }

    const startCanon = flood(playerStart, occWith(boxes0));
    function occWith(boxes) {
      const o = new Uint8Array(N);
      for (let i = 0; i < N; i++) if (fixedOcc[i]) o[i] = 1;
      for (const b of boxes) o[b] = 1;
      return o;
    }
    const k0 = stateKey(startCanon, boxes0);
    nKey[0] = k0;
    best.set(k0, 0);
    heapPush([h0, h0, counter++, 0]);

    let expansions = 0;
    while (heap.length > 0) {
      const [, , , s] = heapPop();
      const g = nG[s];
      if (best.get(nKey[s]) !== g) continue;
      const boxes = nBoxes[s];
      // Goal test.
      let done = true;
      for (const b of boxes) if (!isGoal[b]) { done = false; break; }
      if (done) {
        if (!needExit) return finish(s);
        const o = occWith(boxes);
        flood(nPlayer[s], o);
        let exit = false;
        for (let x = 0; x < w && !exit; x++) if (inRegion(hallwayY * w + x)) exit = true;
        if (exit) return finish(s);
        // Solved but the player is trapped: keep searching for an exit-able
        // solution with the same push count or more.
        if (best.get(nKey[s]) === g) { /* fall through and expand */ }
      }
      if (expansions >= cap) return null;
      expansions++;
      budget.used++;
      if (budget.used >= budget.cap) return null;
      // Expand: mark own boxes, compute the player's region once. Snapshot it:
      // the canonicalisation floods below reuse the shared vis array.
      for (const b of boxes) occ[b] = 1;
      flood(nPlayer[s], occ);
      const region = new Uint8Array(N);
      for (let i = 0; i < N; i++) if (vis[i] === visStamp) region[i] = 1;
      for (let bi = 0; bi < boxes.length; bi++) {
        const b = boxes[bi];
        const bx = b % w, by = Math.floor(b / w);
        for (let d = 0; d < 4; d++) {
          const sx = bx - DX[d], sy = by - DY[d];
          const lx = bx + DX[d], ly = by + DY[d];
          if (sx < 0 || sy < 0 || sx >= w || sy >= h || lx < 0 || ly < 0 || lx >= w || ly >= h) continue;
          const stand = sy * w + sx, land = ly * w + lx;
          if (!region[stand]) continue;
          if (!S.has(land) || occ[land] || dead[land]) continue;
          // Apply the push.
          occ[b] = 0; occ[land] = 1;
          const frozen = freezeDead(land);
          occ[b] = 1; occ[land] = 0;
          if (frozen) continue;
          const nBoxesArr = boxes.slice();
          nBoxesArr[bi] = land;
          nBoxesArr.sort((a, b2) => a - b2);
          const nh = heuristic(nBoxesArr);
          if (nh === Infinity) continue;
          const ng = g + 1;
          // The player ends on the box's old square; canonicalise.
          const canon = flood(b, occWith(nBoxesArr));
          const key = stateKey(canon, nBoxesArr);
          const prev = best.get(key);
          if (prev !== undefined && prev <= ng) continue;
          best.set(key, ng);
          const idx = nBoxes.length;
          nBoxes.push(nBoxesArr);
          nPlayer.push(b);
          nG.push(ng);
          nParent.push(s);
          nFrom.push(b);
          nTo.push(land);
          nDir.push(d);
          nKey.push(key);
          heapPush([ng + nh, nh, counter++, idx]);
        }
      }
      for (const b of boxes) occ[b] = 0;
    }
    return null;

    function finish(s) {
      const pushes = [];
      let cur = s;
      while (nParent[cur] !== -1) {
        pushes.push({ from: nFrom[cur], to: nTo[cur], dir: nDir[cur] });
        cur = nParent[cur];
      }
      pushes.reverse();
      return { pushes, playerEnd: nPlayer[s] };
    }
  }

  // Replay a push list from the current global state, returning the move
  // string and updating `state` ({player, occ}).
  function replay(pushes, state) {
    let out = '';
    for (const p of pushes) {
      const stand = p.from - (DY[p.dir] * w + DX[p.dir]);
      const walk = walkPath(state.player, new Set([stand]), state.occ);
      if (walk === null) return null;
      out += walk + CH_UP[p.dir];
      state.occ[p.from] = 0;
      state.occ[p.to] = 1;
      state.player = p.from;
    }
    return out;
  }

  // ---- decompose into rooms --------------------------------------------------
  const compId = new Int32Array(N).fill(-1);
  const comps = [];
  for (let i = 0; i < N; i++) {
    if (wall[i] || Math.floor(i / w) === hallwayY || compId[i] !== -1) continue;
    const cells = [];
    const stack = [i];
    compId[i] = comps.length;
    while (stack.length > 0) {
      const c = stack.pop();
      cells.push(c);
      const cx = c % w, cy = Math.floor(c / w);
      for (let d = 0; d < 4; d++) {
        const nx = cx + DX[d], ny = cy + DY[d];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const n = ny * w + nx;
        if (wall[n] || ny === hallwayY || compId[n] !== -1) continue;
        compId[n] = comps.length;
        stack.push(n);
      }
    }
    comps.push(cells);
  }
  let decompositionOk = boxesAll.every((b) => Math.floor(b / w) !== hallwayY) && goalsAll.length === boxesAll.length;
  const rooms = [];
  if (decompositionOk) {
    for (const cells of comps) {
      const S = new Set(cells);
      const bs = boxesAll.filter((b) => S.has(b));
      const gs = goalsAll.filter((g) => S.has(g));
      if (bs.length === 0 && gs.length === 0) continue;
      if (bs.length !== gs.length || bs.length === 0) { decompositionOk = false; break; }
      rooms.push({ S, boxes: bs, goals: gs });
    }
  }

  if (decompositionOk && rooms.length > 0) {
    const state = { player, occ: new Uint8Array(N) };
    for (const b of boxesAll) state.occ[b] = 1;
    let moves = '';
    let failed = false;
    // Fixed occupancy for a room = every box not in the room.
    const solvedFinal = new Map(); // comp index -> final boxes
    for (let ri = 0; ri < rooms.length; ri++) {
      const room = rooms[ri];
      const isLast = ri === rooms.length - 1;
      // Walk into the room (best effort; the search also works from outside).
      const walk = walkPath(state.player, room.S, state.occ);
      if (walk !== null) {
        // Track the player's end cell of the walk.
        let p = state.player;
        for (const ch of walk) {
          const d = CH.indexOf(ch);
          p = p + DY[d] * w + DX[d];
        }
        state.player = p;
        moves += walk;
      }
      const fixedOcc = new Uint8Array(N);
      for (let i = 0; i < N; i++) if (state.occ[i] && !room.S.has(i)) fixedOcc[i] = 1;
      const currentBoxes = room.boxes.filter((b) => state.occ[b]);
      let res = pushSearch(room.S, currentBoxes, room.goals, fixedOcc, state.player, false, ROOM_EXPANSION_CAP);
      if (res && !isLast) {
        // Check the player can get back to the hallway afterwards.
        const testOcc = new Uint8Array(N);
        for (let i = 0; i < N; i++) if (fixedOcc[i]) testOcc[i] = 1;
        const finalBoxes = currentBoxes.slice();
        // Apply pushes to a scratch occupancy to find the final box set.
        const scratch = new Set(currentBoxes);
        for (const p of res.pushes) { scratch.delete(p.from); scratch.add(p.to); }
        for (const b of scratch) testOcc[b] = 1;
        flood(res.playerEnd, testOcc);
        let exit = false;
        for (let x = 0; x < w && !exit; x++) if (inRegion(hallwayY * w + x)) exit = true;
        if (!exit) {
          res = pushSearch(room.S, currentBoxes, room.goals, fixedOcc, state.player, true, ROOM_EXPANSION_CAP) ?? res;
        }
      }
      if (!res) { failed = true; break; }
      const seg = replay(res.pushes, state);
      if (seg === null) { failed = true; break; }
      moves += seg;
    }
    if (!failed) return moves;
    // Fall through to the whole-level search with a fresh budget share.
  }

  // ---- whole-level fallback ---------------------------------------------------
  const S = new Set();
  for (let i = 0; i < N; i++) if (!wall[i]) S.add(i);
  const res = pushSearch(S, boxesAll, goalsAll, new Uint8Array(N), player, false, WHOLE_EXPANSION_CAP);
  if (!res) return '';
  const state = { player, occ: new Uint8Array(N) };
  for (const b of boxesAll) state.occ[b] = 1;
  return replay(res.pushes, state) ?? '';
}

function solve(text) {
  const budget = { used: 0, cap: LEVEL_EXPANSION_CAP };
  try {
    return solveLevel(text, budget);
  } catch {
    return '';
  }
}

for (const id of readdirSync(I).sort()) {
  const moves = solve(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
