// Pond Volume & Routing Calculator (GitHub Pages)
// Modes: SCS (CN runoff + triangular hydrograph) OR Inflow (triangular/CSV)
// Geometry: Rect, Circ, Custom stage-area
// Outlet: Orifice + Weir
// Routing: level-pool with bisection on stage to satisfy storage continuity

const g = 9.81;

let chartHydro = null;
let chartStage = null;

let lastStageTable = null;     // {elev[], depth[], area[], storage[], qout[]}
let lastRouting = null;        // {tMin[], Qin[], Qout[], WL[]}

function $(id){ return document.getElementById(id); }

function setMsg(txt){ $("msg").textContent = txt; }

function fmt(x, d=2){
  if(!isFinite(x)) return "–";
  return Number(x).toFixed(d);
}

function clampMin(val, min){
  return (val < min) ? min : val;
}

function parseNumber(id){
  const v = Number($(id).value);
  return isFinite(v) ? v : NaN;
}

function currentMode(){
  return [...document.querySelectorAll("input[name='mode']")].find(r=>r.checked).value;
}

function showHideByMode(){
  const mode = currentMode();
  $("modeSCS").classList.toggle("hidden", mode !== "scs");
  $("modeInflow").classList.toggle("hidden", mode !== "inflow");
}

function showHideGeometry(){
  const t = $("geomType").value;
  $("geomRect").classList.toggle("hidden", t !== "rect");
  $("geomCirc").classList.toggle("hidden", t !== "circ");
  $("geomCustom").classList.toggle("hidden", t !== "custom");
}

function showHideInflowType(){
  const t = $("inflowType").value;
  $("inflowTri").classList.toggle("hidden", t !== "tri");
  $("inflowCsv").classList.toggle("hidden", t !== "csv");
}

function readCustomStageArea(text){
  // returns sorted array of {depth, area}
  const lines = text.split(/\r?\n/).map(s=>s.trim()).filter(Boolean);
  const arr = [];
  for(const ln of lines){
    const parts = ln.split(",").map(s=>s.trim());
    if(parts.length < 2) continue;
    const d = Number(parts[0]);
    const a = Number(parts[1]);
    if(isFinite(d) && isFinite(a) && d >= 0 && a >= 0) arr.push({depth:d, area:a});
  }
  arr.sort((x,y)=>x.depth-y.depth);
  // enforce unique depths
  const out = [];
  for(const it of arr){
    if(out.length && Math.abs(out[out.length-1].depth - it.depth) < 1e-9){
      out[out.length-1] = it;
    } else out.push(it);
  }
  return out;
}

function buildStageStorage(params){
  // params: {bottomElev, maxWLElev, step, geomType, geom...}
  const {bottomElev, maxWLElev, step, geomType} = params;
  const Hmax = maxWLElev - bottomElev;
  if(!(Hmax > 0)) throw new Error("Max water level must be above pond bottom.");

  const n = Math.floor(Hmax / step) + 1;
  const depth = [];
  const elev = [];
  const area = [];
  const storage = [];

  if(geomType === "custom"){
    const table = readCustomStageArea($("customTable").value);
    if(table.length < 2) throw new Error("Custom stage–area table needs at least 2 rows.");

    // We will interpolate area at each depth and integrate trapezoidally for storage
    for(let i=0;i<n;i++){
      const h = Math.min(i*step, Hmax);
      depth.push(h);
      elev.push(bottomElev + h);
      area.push(interpArea(table, h));
    }
    storage.push(0);
    for(let i=1;i<depth.length;i++){
      const dh = depth[i]-depth[i-1];
      const v = storage[i-1] + 0.5*(area[i]+area[i-1])*dh;
      storage.push(v);
    }
    return {depth,elev,area,storage};
  }

  for(let i=0;i<n;i++){
    const h = Math.min(i*step, Hmax);
    depth.push(h);
    elev.push(bottomElev + h);

    if(geomType === "rect"){
      const Lb = parseNumber("lb");
      const Wb = parseNumber("wb");
      const Z = parseNumber("zRect");
      if(!(Lb>0 && Wb>0 && Z>=0)) throw new Error("Rectangular geometry inputs are invalid.");
      const A = (Lb + 2*Z*h) * (Wb + 2*Z*h);
      area.push(A);

      // closed-form volume from 0..h
      const V = (Lb*Wb*h) + (Z*(Lb+Wb)*h*h) + ((4/3)*Z*Z*h*h*h);
      storage.push(V);
    } else if(geomType === "circ"){
      const r0 = parseNumber("r0");
      const Z = parseNumber("zCirc");
      if(!(r0>0 && Z>=0)) throw new Error("Circular geometry inputs are invalid.");
      const r = r0 + Z*h;
      const A = Math.PI*r*r;
      area.push(A);

      const V = Math.PI*(r0*r0*h + r0*Z*h*h + (Z*Z/3)*h*h*h);
      storage.push(V);
    } else {
      throw new Error("Unknown geometry type.");
    }
  }
  return {depth,elev,area,storage};
}

function interpArea(table, h){
  // table: [{depth, area}] sorted
  if(h <= table[0].depth) return table[0].area;
  if(h >= table[table.length-1].depth) return table[table.length-1].area;
  for(let i=1;i<table.length;i++){
    const a = table[i-1], b = table[i];
    if(h >= a.depth && h <= b.depth){
      const t = (h-a.depth)/(b.depth-a.depth);
      return a.area + t*(b.area-a.area);
    }
  }
  return table[table.length-1].area;
}

function outletQ(stageElev, outlet){
  // stageElev: water surface elevation (m)
  // outlet: {useOrifice, orDia, orInv, orCd, useWeir, weirCrest, weirLen, weirCw}
  let q = 0;

  if(outlet.useOrifice){
    const H = stageElev - outlet.orInv;
    if(H > 0){
      const A = Math.PI*(outlet.orDia*outlet.orDia)/4;
      q += outlet.orCd * A * Math.sqrt(2*g*H);
    }
  }

  if(outlet.useWeir){
    const H = stageElev - outlet.weirCrest;
    if(H > 0){
      q += outlet.weirCw * outlet.weirLen * Math.pow(H, 1.5);
    }
  }

  return q;
}

function buildStageDischarge(stage, outlet){
  const qout = stage.elev.map(e=>outletQ(e, outlet));
  return qout;
}

function makeTriHydrograph(Qp, TpMin, TbMin, dtMin){
  // returns arrays tMin[], Q[]
  const Tb = Math.max(TbMin, dtMin);
  const n = Math.floor(Tb/dtMin)+1;
  const t = [];
  const q = [];
  for(let i=0;i<n;i++){
    const tm = i*dtMin;
    t.push(tm);
    let val = 0;
    if(tm <= TpMin){
      val = (TpMin<=0) ? 0 : Qp*(tm/TpMin);
    } else if(tm <= Tb){
      const denom = (Tb - TpMin);
      val = (denom<=0) ? 0 : Qp*(1 - (tm - TpMin)/denom);
    }
    q.push(Math.max(0,val));
  }
  return {tMin:t, Q:q};
}

function parseCsvHydro(text){
  const lines = text.split(/\r?\n/).map(s=>s.trim()).filter(Boolean);
  const t = [];
  const q = [];
  for(const ln of lines){
    const parts = ln.split(",").map(s=>s.trim());
    if(parts.length<2) continue;
    const tm = Number(parts[0]);
    const qm = Number(parts[1]);
    if(isFinite(tm) && isFinite(qm)){
      t.push(tm);
      q.push(qm);
    }
  }
  // sort by time
  const pairs = t.map((x,i)=>({t:x,q:q[i]})).sort((a,b)=>a.t-b.t);
  const outT = [], outQ = [];
  for(const p of pairs){
    if(outT.length && Math.abs(outT[outT.length-1]-p.t)<1e-9){
      outQ[outQ.length-1] = p.q;
    } else {
      outT.push(p.t);
      outQ.push(p.q);
    }
  }
  if(outT.length < 2) throw new Error("CSV hydrograph needs at least 2 points.");
  return {tMin: outT, Q: outQ};
}

function resampleToDt(h, dtMin){
  // h: {tMin[], Q[]} possibly irregular
  // returns uniform dt series from t0..tEnd
  const t0 = h.tMin[0];
  const tEnd = h.tMin[h.tMin.length-1];
  const n = Math.floor((tEnd - t0)/dtMin)+1;
  const t = [];
  const q = [];
  for(let i=0;i<n;i++){
    const tm = t0 + i*dtMin;
    t.push(tm);
    q.push(interp1(h.tMin, h.Q, tm));
  }
  return {tMin:t, Q:q};
}

function interp1(x, y, xi){
  if(xi <= x[0]) return y[0];
  if(xi >= x[x.length-1]) return y[y.length-1];
  for(let i=1;i<x.length;i++){
    if(xi >= x[i-1] && xi <= x[i]){
      const t = (xi-x[i-1])/(x[i]-x[i-1]);
      return y[i-1] + t*(y[i]-y[i-1]);
    }
  }
  return y[y.length-1];
}

function scsRunoffDepthMm(Pmm, CN){
  // NRCS CN runoff depth (mm)
  // S (inches) = 1000/CN - 10 ; Ia = 0.2S
  // convert mm with 1 inch = 25.4mm
  const S_in = (1000/CN) - 10;
  const S_mm = S_in * 25.4;
  const Ia = 0.2 * S_mm;
  if(Pmm <= Ia) return 0;
  const Q = Math.pow(Pmm - Ia, 2) / (Pmm + 0.8*S_mm);
  return Math.max(0, Q);
}

function scsTriInflow(areaHa, runoffMm, TcHr, DurHr, kTb, dtMin){
  // Make a triangular inflow hydrograph with:
  // - Total volume = runoff depth * area
  // - Tp estimated from Tc + storm duration (simple practical)
  // - Tb = kTb * Tp
  //
  // Volume (m3) = runoff(mm)/1000 * area(m2)
  const areaM2 = areaHa * 10000;
  const V = (runoffMm/1000) * areaM2;

  // Tp estimate (hours)
  // A simple robust choice for website sizing:
  // Tp = 0.6*Tc + 0.5*Dur
  const TpHr = 0.6*TcHr + 0.5*DurHr;
  const TpMin = Math.max(1, TpHr * 60);

  const TbMin = Math.max(TpMin + dtMin, kTb*TpMin);

  // For triangle: Volume = 0.5 * Qp * TbSeconds
  const TbSec = TbMin * 60;
  const Qp = (TbSec > 0) ? (2*V / TbSec) : 0;

  const tri = makeTriHydrograph(Qp, TpMin, TbMin, dtMin);
  return { ...tri, meta:{V, Qp, TpMin, TbMin} };
}

function routeLevelPool(stage, qoutArr, inflow, dtMin){
  // stage: {elev[], storage[]} monotonic
  // qoutArr: same length as elev
  // inflow: {tMin[], Q[]} uniform dt
  // dtMin: routing step
  const dtSec = dtMin*60;

  const t = inflow.tMin;
  const Qin = inflow.Q;

  // initial stage = pond bottom (storage=0)
  let WL = stage.elev[0];
  let S = stage.storage[0];
  let O = outletFromStage(stage, qoutArr, WL);

  const WLs = [WL];
  const Qouts = [O];

  for(let i=1;i<t.length;i++){
    const I1 = Qin[i-1], I2 = Qin[i];
    const Iavg = 0.5*(I1+I2);
    const O1 = Qouts[i-1];

    // Continuity target storage at next step:
    // S2 = S1 + (Iavg - Oavg)*dt, but Oavg depends on WL2
    // We'll solve by bisection on WL2:
    const targetFn = (WL2)=>{
      const S2 = storageFromStage(stage, WL2);
      const O2 = outletFromStage(stage, qoutArr, WL2);
      const Oavg = 0.5*(O1 + O2);
      const Scont = S + (Iavg - Oavg)*dtSec;
      return S2 - Scont;
    };

    // Search bounds: bottom..top stage
    const lo = stage.elev[0];
    const hi = stage.elev[stage.elev.length-1];

    let WL2 = bisect(targetFn, lo, hi, 40);
    WL2 = Math.max(lo, Math.min(hi, WL2));

    const S2 = storageFromStage(stage, WL2);
    const O2 = outletFromStage(stage, qoutArr, WL2);

    WLs.push(WL2);
    Qouts.push(O2);

    WL = WL2; S = S2;
  }

  return {tMin:t, Qin:Qin, Qout:Qouts, WL:WLs};
}

function bisect(f, lo, hi, iters=40){
  let flo = f(lo);
  let fhi = f(hi);

  // If not bracketed, return whichever gives smaller abs residual (robust fallback)
  if(!(flo===0 || fhi===0) && (flo*fhi > 0)){
    let bestX = lo, bestF = Math.abs(flo);
    const mid = 0.5*(lo+hi);
    const fmid = f(mid);
    if(Math.abs(fmid) < bestF){ bestF=Math.abs(fmid); bestX=mid; }
    if(Math.abs(fhi) < bestF){ bestF=Math.abs(fhi); bestX=hi; }
    return bestX;
  }

  let a=lo, b=hi, fa=flo, fb=fhi;
  for(let i=0;i<iters;i++){
    const m = 0.5*(a+b);
    const fm = f(m);
    if(fm===0) return m;
    if(fa*fm <= 0){
      b=m; fb=fm;
    } else {
      a=m; fa=fm;
    }
  }
  return 0.5*(a+b);
}

function storageFromStage(stage, elev){
  // linear interpolate storage
  return interp1(stage.elev, stage.storage, elev);
}

function outletFromStage(stage, qoutArr, elev){
  return interp1(stage.elev, qoutArr, elev);
}

function maxOf(arr){
  return arr.reduce((m,x)=>Math.max(m,x), -Infinity);
}

function sumTrapz(tMin, q){
  // integrate q(t) over time in seconds using trapezoids, returns volume (m3)
  let V = 0;
  for(let i=1;i<tMin.length;i++){
    const dt = (tMin[i]-tMin[i-1])*60;
    V += 0.5*(q[i]+q[i-1])*dt;
  }
  return V;
}

function updateCharts(stage, routing){
  // Hydrograph chart
  const labels = routing.tMin.map(x=>x.toFixed(0));
  const dataHydro = {
    labels,
    datasets: [
      { label: "Qin (m³/s)", data: routing.Qin, tension: 0.15 },
      { label: "Qout (m³/s)", data: routing.Qout, tension: 0.15 },
      { label: "WL (m)", data: routing.WL, yAxisID: "y2", tension: 0.15 },
    ]
  };

  const ctxH = $("chartHydro").getContext("2d");
  if(chartHydro) chartHydro.destroy();
  chartHydro = new Chart(ctxH, {
    type:"line",
    data:dataHydro,
    options:{
      responsive:true,
      interaction:{mode:"index", intersect:false},
      plugins:{legend:{labels:{color:"#e7eefc"}}},
      scales:{
        x:{ ticks:{color:"#a8b7d6"}, grid:{color:"rgba(38,54,87,.35)"} },
        y:{ ticks:{color:"#a8b7d6"}, grid:{color:"rgba(38,54,87,.35)"} , title:{display:true, text:"Flow (m³/s)", color:"#a8b7d6"}},
        y2:{ position:"right", ticks:{color:"#a8b7d6"}, grid:{drawOnChartArea:false}, title:{display:true, text:"Water Level (m)", color:"#a8b7d6"} }
      }
    }
  });

  // Stage chart
  const dataStage = {
    labels: stage.elev.map(e=>e.toFixed(2)),
    datasets: [
      { label:"Storage (m³)", data: stage.storage, tension:0.15, yAxisID:"y" },
      { label:"Qout (m³/s)", data: stage.qout, tension:0.15, yAxisID:"y2" }
    ]
  };

  const ctxS = $("chartStage").getContext("2d");
  if(chartStage) chartStage.destroy();
  chartStage = new Chart(ctxS, {
    type:"line",
    data:dataStage,
    options:{
      responsive:true,
      interaction:{mode:"index", intersect:false},
      plugins:{legend:{labels:{color:"#e7eefc"}}},
      scales:{
        x:{ ticks:{color:"#a8b7d6"}, grid:{color:"rgba(38,54,87,.35)"}, title:{display:true, text:"Stage Elevation (m)", color:"#a8b7d6"} },
        y:{ ticks:{color:"#a8b7d6"}, grid:{color:"rgba(38,54,87,.35)"}, title:{display:true, text:"Storage (m³)", color:"#a8b7d6"} },
        y2:{ position:"right", ticks:{color:"#a8b7d6"}, grid:{drawOnChartArea:false}, title:{display:true, text:"Outflow (m³/s)", color:"#a8b7d6"} }
      }
    }
  });
}

function updateKPIs(levels, stage, routing){
  $("kpiBottom").textContent = fmt(levels.bottomElev,2);
  $("kpiHWL").textContent = fmt(levels.maxWLElev,2);

  const maxWL = maxOf(routing.WL);
  $("kpiMaxWL").textContent = fmt(maxWL,2);

  const fbOk = maxWL <= levels.maxWLElev + 1e-9;
  $("kpiFB").textContent = fbOk ? "YES" : "NO";

  // storage at HWLmax
  const Vtot = stage.storage[stage.storage.length-1];

  // storage below IL: from bottom..IL
  const Vbelow = (levels.il <= levels.bottomElev) ? 0 : storageFromStage(stage, Math.min(levels.il, levels.maxWLElev));
  const Vactive = Math.max(0, Vtot - Vbelow);

  $("kpiVtot").textContent = fmt(Vtot,0);
  $("kpiVbelow").textContent = fmt(Vbelow,0);
  $("kpiVactive").textContent = fmt(Vactive,0);

  $("kpiQout").textContent = fmt(maxOf(routing.Qout),2);
}

function buildDownloadCsv(stage, routing){
  // Stage table CSV
  const stageLines = ["Depth(m),Elev(m),Area(m2),Storage(m3),Qout(m3/s)"];
  for(let i=0;i<stage.depth.length;i++){
    stageLines.push([
      stage.depth[i].toFixed(3),
      stage.elev[i].toFixed(3),
      stage.area[i].toFixed(3),
      stage.storage[i].toFixed(3),
      stage.qout[i].toFixed(6)
    ].join(","));
  }

  // Routing table CSV
  const routeLines = ["t(min),Qin(m3/s),Qout(m3/s),WL(m)"];
  for(let i=0;i<routing.tMin.length;i++){
    routeLines.push([
      routing.tMin[i].toFixed(3),
      routing.Qin[i].toFixed(6),
      routing.Qout[i].toFixed(6),
      routing.WL[i].toFixed(4)
    ].join(","));
  }

  const all = [
    "### STAGE TABLE",
    ...stageLines,
    "",
    "### ROUTING TABLE",
    ...routeLines
  ].join("\n");

  return all;
}

function downloadText(filename, text){
  const blob = new Blob([text], {type:"text/csv;charset=utf-8"});
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  a.remove();
  URL.revokeObjectURL(url);
}

function run(){
  try{
    setMsg("Running...");

    // Levels
    const egl = parseNumber("egl");
    const il = parseNumber("il");
    const belowIL = clampMin(parseNumber("belowIL"), 0.30);
    $("belowIL").value = belowIL.toFixed(2);

    const freeboard = clampMin(parseNumber("freeboard"), 0.0);
    const step = clampMin(parseNumber("stageStep"), 0.01);
    const dtMin = clampMin(parseNumber("dtMin"), 0.25);

    if(!isFinite(egl) || !isFinite(il)) throw new Error("EGL and IL must be valid numbers.");

    const bottomElev = il - belowIL;
    const maxWLElev = egl - freeboard;

    if(!(maxWLElev > bottomElev)){
      throw new Error("Invalid levels: (EGL - Freeboard) must be higher than (IL - DepthBelowIL).");
    }

    // Stage-storage
    const geomType = $("geomType").value;
    const stage = buildStageStorage({bottomElev, maxWLElev, step, geomType});
    // Outlet
    const outlet = {
      useOrifice: $("useOrifice").checked,
      orDia: parseNumber("orDia"),
      orInv: parseNumber("orInv"),
      orCd: parseNumber("orCd"),
      useWeir: $("useWeir").checked,
      weirCrest: parseNumber("weirCrest"),
      weirLen: parseNumber("weirLen"),
      weirCw: parseNumber("weirCw")
    };
    if(!outlet.useOrifice && !outlet.useWeir){
      throw new Error("Enable at least one outlet (Orifice or Weir).");
    }

    // Discharge curve
    const qout = buildStageDischarge(stage, outlet);
    const stageFull = {...stage, qout};

    // Inflow
    let inflowRaw = null;
    if(currentMode() === "scs"){
      const areaHa = parseNumber("areaHa");
      const cn = parseNumber("cn");
      const pmm = parseNumber("pmm");
      const durHr = parseNumber("durHr");
      const tcHr = parseNumber("tcHr");
      const kTb = parseNumber("kTb");

      if(!(areaHa>0 && cn>=30 && cn<=98 && pmm>=0 && durHr>0 && tcHr>0 && kTb>=1.5)){
        throw new Error("SCS inputs are invalid. Check area, CN, P, duration, Tc, and k.");
      }

      const runoffMm = scsRunoffDepthMm(pmm, cn);
      inflowRaw = scsTriInflow(areaHa, runoffMm, tcHr, durHr, kTb, dtMin);

      // ensure dt uniform
      inflowRaw = resampleToDt(inflowRaw, dtMin);

      const Vrunoff = sumTrapz(inflowRaw.tMin, inflowRaw.Q);
      setMsg(`SCS runoff depth = ${fmt(runoffMm,1)} mm | Inflow volume ≈ ${fmt(Vrunoff,0)} m³`);
    } else {
      if($("inflowType").value === "tri"){
        const Qp = parseNumber("qp");
        const TpMin = parseNumber("tpMin");
        const TbMin = parseNumber("tbMin");
        if(!(Qp>=0 && TpMin>0 && TbMin>0 && TbMin>=TpMin)){
          throw new Error("Triangular inflow inputs are invalid (Tb must be ≥ Tp).");
        }
        inflowRaw = makeTriHydrograph(Qp, TpMin, TbMin, dtMin);
      } else {
        const h = parseCsvHydro($("csvText").value);
        inflowRaw = resampleToDt(h, dtMin);
      }
      const V = sumTrapz(inflowRaw.tMin, inflowRaw.Q);
      setMsg(`Inflow volume ≈ ${fmt(V,0)} m³ (from provided hydrograph)`);
    }

    // Routing
    const routing = routeLevelPool(stageFull, stageFull.qout, inflowRaw, dtMin);

    // Save for download
    lastStageTable = stageFull;
    lastRouting = routing;
    $("btnDownload").disabled = false;

    // KPIs & Charts
    updateKPIs({bottomElev, maxWLElev, il}, stageFull, routing);
    updateCharts(stageFull, routing);

    // FB message
    const maxWL = maxOf(routing.WL);
    const fbOk = maxWL <= maxWLElev + 1e-9;
    setMsg(fbOk
      ? `PASS: Max routed WL (${fmt(maxWL,2)} m) ≤ HWLmax (${fmt(maxWLElev,2)} m).`
      : `FAIL: Max routed WL (${fmt(maxWL,2)} m) > HWLmax (${fmt(maxWLElev,2)} m). Increase storage/outlet or revise freeboard.`
    );

  } catch(e){
    setMsg(e.message || String(e));
    $("btnDownload").disabled = true;
  }
}

function init(){
  // mode toggle
  document.querySelectorAll("input[name='mode']").forEach(r=>{
    r.addEventListener("change", showHideByMode);
  });
  $("geomType").addEventListener("change", showHideGeometry);
  $("inflowType").addEventListener("change", showHideInflowType);

  $("btnRun").addEventListener("click", run);
  $("btnDownload").addEventListener("click", ()=>{
    if(!lastStageTable || !lastRouting) return;
    const csv = buildDownloadCsv(lastStageTable, lastRouting);
    downloadText("pond_stage_and_routing.csv", csv);
  });

  $("btnLoadExample").addEventListener("click", ()=>{
    $("csvText").value = `0,0
10,0.15
20,0.45
30,0.90
40,1.20
60,0.85
90,0.35
120,0`;
  });

  showHideByMode();
  showHideGeometry();
  showHideInflowType();
  setMsg("Ready. Enter inputs then click RUN CALCULATION.");
}

init();
