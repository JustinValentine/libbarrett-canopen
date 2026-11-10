"use strict";
const original=JSON.parse(document.getElementById('model-data').textContent);
const clone=x=>JSON.parse(JSON.stringify(x));
let model=clone(original),q=model.moving.map(()=>0),selected=Math.min(2,q.length-1),unit='deg',sweeping=false,sweepBase=0,sweepStart=0,sweepGeneration=0;
let result=DHMath.fk(model,q), dirty=false, rafPending=false;
const $=id=>document.getElementById(id), TAU=Math.PI*2;
const colors=['#288e92','#36848c','#267f85','#3d9290','#cb9a47','#708bc2','#6c88bc','#8599c0'];
const camera={yaw:-1.05,pitch:.30,center:[0,0,.45],scale:460};
const canvas=$('scene'),ctx=canvas.getContext('2d');
let width=600,height=650,hits=[],labelHits=[];
const add=(a,b)=>a.map((v,i)=>v+b[i]), sub=(a,b)=>a.map((v,i)=>v-b[i]);
const times=(a,s)=>a.map(v=>v*s),dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
const norm=a=>Math.hypot(...a);
const clean=(v,d=3)=>Math.abs(v)<.5*10**(-d)?(0).toFixed(d):v.toFixed(d);
const fmt=v=>Number(v.toPrecision(12)).toString();
const configNumber=v=>{const text=String(v);return /[.e]/i.test(text)?text:text+'.0';};
const shown=v=>unit==='deg'?v*180/Math.PI:v;
const radians=v=>unit==='deg'?v*Math.PI/180:v;

function kinematicsText() {
  let text=original.source.slice(...original.span);
  // Replace only edited numeric literals. Preserve comments, unknown settings,
  // and every unedited digit in the source. Insert absent optional offsets
  // only when edited, immediately before their row's closing brace.
  original.fields.slice().sort((a,b)=>b.span[0]-a.span[0]).forEach(field=>{
    const row=field.row<q.length?model.moving[field.row]:model.tool;
    const before=field.row<q.length?original.moving[field.row]:original.tool;
    if(row[field.key]===before[field.key])return;
    const [start,end]=field.span.map(v=>v-original.span[0]);
    const replacement=field.missing?` ${field.key} = ${configNumber(row[field.key])}; `:configNumber(row[field.key]);
    text=text.slice(0,start)+replacement+text.slice(end);
  });
  return text;
}
function exportedConfig(){return original.source.slice(0,original.span[0])+kinematicsText()+original.source.slice(original.span[1]);}
function select(i){if(!Number.isInteger(i)||i<0||i>q.length)return;selected=i; stopSweep(); update();}
function stopSweep(){sweeping=false;sweepGeneration++; $('sweep').textContent='▶ Sweep selected joint';$('sweep').classList.remove('active');}
function validationState(){
  const invalid=!!document.querySelector('input[aria-invalid="true"]');
  $('download').disabled=invalid;$('copy').disabled=invalid;
  $('sweep').disabled=invalid||selected===q.length;
  $('input-error').hidden=!invalid;
}
function markValid(input,valid){
  input.classList.toggle('invalid',!valid);
  input.setAttribute('aria-invalid',String(!valid));
  input.setCustomValidity(valid?'':'Enter a finite number.');
}
function readNumber(input){
  const value=input.valueAsNumber;
  const valid=Number.isFinite(value)&&Number.isFinite(value*1000);
  markValid(input,valid);validationState();return valid?value:null;
}
function restoreOnBlur(input){input.addEventListener('blur',()=>{if(input.getAttribute('aria-invalid')==='true')update();});}
function buildControls(){
  $('joints').replaceChildren(); $('dh-rows').replaceChildren();
  model.moving.forEach((row,i)=>{
    const card=document.createElement('div');card.className='joint';card.dataset.joint=i;
    const head=document.createElement('div');head.className='joint-head';
    const name=document.createElement('button');name.className='joint-name';name.textContent=`J${i+1}`;name.setAttribute('aria-label',`Select joint J${i+1}`);
    if(i===model.passive){const tag=document.createElement('span');tag.className='passive';tag.textContent='PASSIVE';name.append(tag);}
    const number=document.createElement('input');number.type='number';number.step=unit==='deg'?'1':'0.01';number.setAttribute('aria-label',`J${i+1} angle in ${unit}`);number.id=`q-number-${i}`;
    const slider=document.createElement('input');slider.type='range';slider.min=unit==='deg'?-180:-Math.PI;slider.max=unit==='deg'?180:Math.PI;slider.step=unit==='deg'?.1:.001;slider.id=`q-slider-${i}`;slider.setAttribute('aria-label',`J${i+1} angle slider`);
    head.append(name,number);card.append(head,slider);
    card.addEventListener('click',e=>{if(e.target.tagName!=='INPUT')select(i);});
    [number,slider].forEach(input=>input.addEventListener('input',()=>{
      stopSweep();const value=readNumber(input);if(value===null)return;
      selected=i;q[i]=radians(value);update(input);
    }));
    restoreOnBlur(number);
    $('joints').append(card);
  });
  [...model.moving,model.tool].forEach((row,i)=>{
    const tr=document.createElement('tr');tr.dataset.row=i;if(i===model.passive)tr.className='passive-row';
    const label=document.createElement('td'),button=document.createElement('button');button.className='row-select';button.textContent=i===model.moving.length?'Tool':`J${i+1}`;button.setAttribute('aria-label',`Inspect ${button.textContent} frame`);button.addEventListener('click',()=>select(i));label.append(button);tr.append(label);
    ['alpha_pi','theta_pi','a','d'].forEach(key=>{
      const angle=key.endsWith('_pi');
      const td=document.createElement('td'),input=document.createElement('input');input.type='number';input.step=angle?.05:1;input.id=`dh-${i}-${key}`;input.dataset.key=key;input.dataset.row=i;input.setAttribute('aria-label',`${i===model.moving.length?'Tool':'J'+(i+1)} ${key}${angle?' in multiples of pi':' in millimeters'}`);
      if(key==='theta_pi')input.title='Fixed angular offset in multiples of π; included in config exports. Joint sliders add motion to this offset.';
      input.addEventListener('input',()=>{
        stopSweep();const value=readNumber(input);if(value===null)return;
        row[key]=angle?value:value/1000;
        selected=i;update(input);
      });restoreOnBlur(input);td.append(input);tr.append(td);
    });$('dh-rows').append(tr);
  });
}
function update(origin=null){
  result=DHMath.fk(model,q);
  dirty=JSON.stringify([model.moving,model.tool])!==JSON.stringify([original.moving,original.tool]);
  model.moving.forEach((_,i)=>{
    const value=shown(q[i]),n=$(`q-number-${i}`),s=$(`q-slider-${i}`);
    if(n!==origin){n.value=clean(value,unit==='deg'?1:3);markValid(n,true);}
    const bound=unit==='deg'?180:Math.PI;s.min=Math.min(-bound,value);s.max=Math.max(bound,value);s.value=value;
    n.closest('.joint').classList.toggle('selected',i===selected);
  });
  [...model.moving,model.tool].forEach((row,i)=>{
    const before=i<model.moving.length?original.moving[i]:original.tool;
    ['alpha_pi','theta_pi','a','d'].forEach(key=>{
      const input=$(`dh-${i}-${key}`);
      const value=row[key]*(key.endsWith('_pi')?1:1000);
      if(input!==origin){input.value=fmt(value);markValid(input,true);}
      input.classList.toggle('changed',row[key]!==before[key]);
    });
    document.querySelector(`[data-row="${i}"]`).classList.toggle('selected',i===selected);
  });
  const zeros=model.moving.map((r,i)=>r.a===0&&r.d===0&&r.alpha_pi===0?i+1:null).filter(Boolean);
  const warnings=model.warnings.slice();
  if(zeros.length)warnings.push(`Zero DH rows: ${zeros.map(i=>'J'+i).join(', ')}. Their frame origins coincide and consecutive rows share a rotation axis; θ still rotates the frame.`);
  if(model.calibrated===false)warnings.push('This config is marked kinematics_calibrated = false. Geometry requires measurement and verification.');
  $('notice').textContent=warnings.join(' ')||'Inspect the model at multiple known poses. The DH convention matches libbarrett; physical dimensions still require measurement.';
  $('notice').classList.toggle('info',!warnings.length);
  $('model-state').textContent=dirty?'Edited':'Original';
  const tool=DHMath.point(result.tool);
  ['x','y','z'].forEach((key,i)=>{$(`tool-${key}`).replaceChildren(document.createTextNode(clean(tool[i]*1000,1)+' '));const mm=document.createElement('small');mm.textContent='mm';$(`tool-${key}`).append(mm);});
  const isTool=selected===model.moving.length;
  const t=isTool?result.tool:result.frames[selected+1];
  $('selection-caption').textContent=isTool?'Toolplate frame':`J${selected+1}${selected===model.passive?' · passive joint':''}`;
  $('selection-subtitle').textContent=isTool?'Fixed transform after the last moving link':`Rotation about z${selected} of ${selected===0?'the base frame':'frame F'+selected}`;
  $('inspector-title').textContent=isTool?'Toolplate in world':`J${selected+1} rotation axis in world`;
  const axis=isTool?DHMath.axis(t,2):result.joints[selected].axis;
  const pos=isTool?DHMath.point(t):result.joints[selected].origin;
  $('axis-values').replaceChildren();
  for(const text of [`Origin · mm  [${pos.map(v=>clean(v*1000,1)).join(', ')}]`,`Axis · unit  [${axis.map(v=>clean(v,3)).join(', ')}]`]){const d=document.createElement('div');d.textContent=text;$('axis-values').append(d);}
  $('matrix-title').textContent=isTool?'Toolplate → world transform':`F${selected+1} → world transform`;
  $('matrix').replaceChildren(...t.flat().map(v=>{const e=document.createElement('span');e.textContent=clean(v,4);return e;}));
  $('config-preview').value='kinematics: '+kinematicsText()+';';
  validationState();
  requestDraw();
}
function basis(){
  const sy=Math.sin(camera.yaw),cy=Math.cos(camera.yaw),sp=Math.sin(camera.pitch),cp=Math.cos(camera.pitch);
  return {right:[-sy,cy,0],up:[-cy*sp,-sy*sp,cp],out:[cy*cp,sy*cp,sp]};
}
function project(p){const b=basis(),v=sub(p,camera.center);return [width/2+dot(v,b.right)*camera.scale,height/2-dot(v,b.up)*camera.scale,dot(v,b.out)];}
function fit(){
  const b=basis(),points=[...result.frames.map(m=>DHMath.point(m)),DHMath.point(result.tool),result.toolLink.corner,...result.links.map(l=>l.corner)];
  if($('ghost').checked){const ghost=DHMath.fk(original,q);points.push(...ghost.frames.map(m=>DHMath.point(m)),DHMath.point(ghost.tool),ghost.toolLink.corner,...ghost.links.map(l=>l.corner));}
  const lo=[0,1,2].map(i=>Math.min(...points.map(p=>p[i]))),hi=[0,1,2].map(i=>Math.max(...points.map(p=>p[i])));
  camera.center=lo.map((v,i)=>(v+hi[i])/2);
  const horizontal=points.map(p=>dot(sub(p,camera.center),b.right)),vertical=points.map(p=>dot(sub(p,camera.center),b.up));
  const dx=Math.max(...horizontal)-Math.min(...horizontal),dy=Math.max(...vertical)-Math.min(...vertical);
  camera.scale=Math.min((width-140)/Math.max(.28,dx+.20),(height-190)/Math.max(.35,dy+.23));
  camera.scale=Math.max(20,camera.scale);requestDraw();
}
function requestDraw(){if(!rafPending){rafPending=true;requestAnimationFrame(()=>{rafPending=false;draw();});}}
function draw(){
  ctx.clearRect(0,0,width,height);hits=[];labelHits=[];
  // Orthographic projection preserves relative lengths in each chosen view.
  const primitives=[],overlays=[],axisHeads=[];
  const line=(a,b,color,w=1,dash=[],priority=0)=>{const p=project(a),r=project(b);primitives.push({depth:(p[2]+r[2])/2+priority,paint(){ctx.beginPath();ctx.moveTo(p[0],p[1]);ctx.lineTo(r[0],r[1]);ctx.strokeStyle=color;ctx.lineWidth=w;ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([]);}});};
  const disc=(p,r,color,stroke='#fff')=>{const s=project(p);primitives.push({depth:s[2]+.0002,paint(){ctx.beginPath();ctx.arc(s[0],s[1],r,0,TAU);ctx.fillStyle=color;ctx.fill();ctx.lineWidth=1.5;ctx.strokeStyle=stroke;ctx.stroke();}});};
  const arrow=(a,b,color,w=3,priority=0,jointAxis=false)=>{
    const p=project(a),r=project(b),dx=r[0]-p[0],dy=r[1]-p[1],n=Math.hypot(dx,dy);
    const head=jointAxis&&n<12?p:r;
    if(jointAxis||n>=2)axisHeads.push({x:head[0]-16,y:head[1]-16,w:32,h:32});
    // Draw axes over the links, with a white outline and screen-sized heads.
    // Looking straight down an axis uses the usual dot/cross direction symbol.
    overlays.push({priority,paint(){
      ctx.save();ctx.setLineDash([]);
      if(jointAxis&&n<12){
        const radius=w>3?11:8;
        ctx.beginPath();ctx.arc(p[0],p[1],radius,0,TAU);ctx.fillStyle='#fff';ctx.fill();
        ctx.strokeStyle=color;ctx.lineWidth=w;ctx.stroke();
        if(r[2]>=p[2]){ctx.beginPath();ctx.arc(p[0],p[1],3,0,TAU);ctx.fillStyle=color;ctx.fill();}
        else{ctx.beginPath();ctx.moveTo(p[0]-4,p[1]-4);ctx.lineTo(p[0]+4,p[1]+4);ctx.moveTo(p[0]-4,p[1]+4);ctx.lineTo(p[0]+4,p[1]-4);ctx.lineWidth=2.5;ctx.stroke();}
      }else if(n>=2){
        const u=[dx/n,dy/n],sz=Math.min(n,Math.max(10,w*3.5));
        ctx.beginPath();ctx.moveTo(p[0],p[1]);ctx.lineTo(r[0],r[1]);
        ctx.strokeStyle='#fff';ctx.lineWidth=w+4;ctx.stroke();ctx.strokeStyle=color;ctx.lineWidth=w;ctx.stroke();
        ctx.beginPath();ctx.moveTo(r[0],r[1]);
        ctx.lineTo(r[0]-u[0]*sz-u[1]*sz*.55,r[1]-u[1]*sz+u[0]*sz*.55);
        ctx.lineTo(r[0]-u[0]*sz+u[1]*sz*.55,r[1]-u[1]*sz-u[0]*sz*.55);ctx.closePath();
        ctx.strokeStyle='#fff';ctx.lineWidth=3;ctx.stroke();ctx.fillStyle=color;ctx.fill();
      }
      ctx.restore();
    }});
  };
  // Ground grid, 100 mm subdivisions near the base.
  const base=DHMath.point(result.frames[0]);const extent=Math.max(.6,Math.min(4,2*height/camera.scale));
  const step=extent>2?.5:.1, count=Math.ceil(extent/step);
  for(let j=-count;j<=count;j++){
    const off=j*step,col=j===0?'#ccdcd9':'#e6eeeb';
    line([base[0]+off,base[1]-extent,base[2]],[base[0]+off,base[1]+extent,base[2]],col,1,[], -10);
    line([base[0]-extent,base[1]+off,base[2]],[base[0]+extent,base[1]+off,base[2]],col,1,[], -10);
  }
  const links=(fk,ghost=false)=>{
    [...fk.links,fk.toolLink].forEach((l,i)=>{
      const tool=i===model.moving.length,c=ghost?'#a7b8b688':(tool?'#637482':colors[i%colors.length]);
      const w=ghost?3:(tool?7:10);
      // The elbow is the Tz(d) then Tx(a) construction, not a mesh of the arm.
      line(l.start,l.corner,c,w,ghost?[4,5]:[],ghost?-.1:0);
      line(l.corner,l.end,c,w,ghost?[4,5]:[],ghost?-.1:0);
      if(!ghost && norm(sub(l.start,l.corner))>.001 && norm(sub(l.end,l.corner))>.001)disc(l.corner,3.5,c);
    });
  };
  if($('ghost').checked)links(DHMath.fk(original,q),true);
  links(result);
  const frameLength=Number($('frame-size').value)/1000;
  const frame=(t,label,selectedFrame=false)=>{
    const p=DHMath.point(t),l=frameLength*(selectedFrame?1:0.72);
    ['#c93443','#168044','#2168bb'].forEach((c,k)=>arrow(p,add(p,times(DHMath.axis(t,k),l)),c,selectedFrame?3:2,.006));
  };
  if($('all-frames').checked){result.frames.forEach((t,i)=>frame(t,'F'+i,i===selected+1));frame(result.tool,'Tool',selected===model.moving.length);}
  else {frame(result.frames[Math.min(selected,model.moving.length)],'Previous',false);frame(selected===model.moving.length?result.tool:result.frames[selected+1],'Selected',true);}
  let selectedAxisTip=null;
  result.joints.forEach((joint,i)=>{
    const sel=i===selected;
    if($('axes').checked||sel){
      const length=frameLength*(sel?2.6:1.5),color=sel?'#c65c00':'#397578';
      const tip=add(joint.origin,times(joint.axis,length*.72));
      line(add(joint.origin,times(joint.axis,-length*.42)),joint.origin,color,sel?2.5:1.5,[6,5],.015);
      arrow(joint.origin,tip,color,sel?4.5:2.5,sel?2:1,true);
      if(sel)selectedAxisTip=tip;
    }
    disc(joint.origin,sel?10:7,sel?'#c65c00':(i===model.passive?'#a76b14':'#397578'));
    const p=project(joint.origin);hits.push({x:p[0],y:p[1],i});
  });
  disc(DHMath.point(result.tool),6,'#485c73');
  const toolHit=project(DHMath.point(result.tool));hits.push({x:toolHit[0],y:toolHit[1],i:q.length});
  primitives.sort((a,b)=>a.depth-b.depth);ctx.lineCap='round';ctx.lineJoin='round';primitives.forEach(p=>p.paint());
  overlays.sort((a,b)=>a.priority-b.priority).forEach(p=>p.paint());
  if($('labels').checked){
    const occupied=axisHeads.slice();
    const label=(pos,text,color='#25494c',highlight=false,index=null,small=false)=>{
      const p=project(pos),h=small?26:32;
      ctx.font=`700 ${small?13:16}px -apple-system,BlinkMacSystemFont,Segoe UI,sans-serif`;
      const w=ctx.measureText(text).width+20,candidates=[];
      // Separate even coincident joints into individual, clickable badges.
      // Search nearby placements so labels stay inside the view and apart.
      for(const gap of [22,58,94,130]){
        for(const [dx,dy] of [[gap,-h-12],[gap,12],[-w-gap,-h-12],[-w-gap,12],[-w/2,-h-gap],[-w/2,gap]]){
          const x=Math.max(10,Math.min(width-w-10,p[0]+dx));
          const y=Math.max(100,Math.min(height-h-94,p[1]+dy));
          const overlap=occupied.reduce((sum,r)=>sum+Math.max(0,Math.min(x+w+7,r.x+r.w)-Math.max(x-7,r.x))*Math.max(0,Math.min(y+h+7,r.y+r.h)-Math.max(y-7,r.y)),0);
          candidates.push({x,y,w,h,score:overlap*100+Math.hypot(x+w/2-p[0],y+h/2-p[1])});
        }
      }
      candidates.sort((a,b)=>a.score-b.score);const box=candidates[0];occupied.push(box);
      const end=[Math.max(box.x,Math.min(box.x+w,p[0])),Math.max(box.y,Math.min(box.y+h,p[1]))];
      ctx.beginPath();ctx.moveTo(p[0],p[1]);ctx.lineTo(...end);
      ctx.strokeStyle='#fff';ctx.lineWidth=5;ctx.stroke();ctx.strokeStyle=color;ctx.lineWidth=highlight?2.5:1.5;ctx.stroke();
      ctx.beginPath();ctx.roundRect(box.x,box.y,w,h,7);ctx.fillStyle=highlight?'#fff1df':'#fff';ctx.fill();
      ctx.strokeStyle=highlight?'#c65c00':'#8aa4a6';ctx.lineWidth=highlight?2.5:1.5;ctx.stroke();
      ctx.fillStyle=color;ctx.textBaseline='middle';ctx.fillText(text,box.x+10,box.y+h/2);ctx.textBaseline='alphabetic';
      if(index!==null)labelHits.push({...box,i:index});
    };
    if(selectedAxisTip)label(selectedAxisTip,`J${selected+1} axis · +Z${selected}`,'#9c4700',true,selected,true);
    result.joints.forEach((joint,i)=>label(joint.origin,`J${i+1}${i===model.passive?' · passive':''}`,i===selected?'#9c4700':'#25494c',i===selected,i));
    label(DHMath.point(result.tool),'TOOL','#354f6c',selected===q.length,q.length,true);
    // Explicit resulting frame labels disambiguate outgoing frames from joint axes.
    const t=selected===model.moving.length?result.tool:result.frames[selected+1];
    if(selected<q.length)label(DHMath.point(t),`Frame F${selected+1}`,'#435d72',false,null,true);
  }
  // Orientation triad in screen corner.
  const b=basis(),anchor=[width-48,height-61];
  [[1,0,0],[0,1,0],[0,0,1]].forEach((a,i)=>{const end=[anchor[0]+dot(a,b.right)*27,anchor[1]-dot(a,b.up)*27];ctx.beginPath();ctx.moveTo(...anchor);ctx.lineTo(...end);ctx.strokeStyle=['#d95e63','#48996d','#5389c7'][i];ctx.lineWidth=2;ctx.stroke();ctx.fillStyle=ctx.strokeStyle;ctx.font='10px sans-serif';ctx.fillText('XYZ'[i],end[0]+3,end[1]-2);});
}
function resize(){const r=$('viewport').getBoundingClientRect(),dpr=window.devicePixelRatio||1;width=r.width;height=r.height;canvas.width=Math.round(width*dpr);canvas.height=Math.round(height*dpr);ctx.setTransform(dpr,0,0,dpr,0,0);requestDraw();}
let drag=null;
canvas.addEventListener('pointerdown',e=>{canvas.setPointerCapture(e.pointerId);drag={x:e.clientX,y:e.clientY,firstX:e.clientX,firstY:e.clientY,pan:e.shiftKey||e.button===2};});
canvas.addEventListener('pointermove',e=>{
  if(!drag)return;const dx=e.clientX-drag.x,dy=e.clientY-drag.y;drag.x=e.clientX;drag.y=e.clientY;
  if(drag.pan){const b=basis();camera.center=add(camera.center,add(times(b.right,-dx/camera.scale),times(b.up,dy/camera.scale)));}
  else{camera.yaw-=dx*.008;camera.pitch=Math.max(-Math.PI/2,Math.min(Math.PI/2,camera.pitch+dy*.008));document.querySelectorAll('#views button').forEach(b=>b.classList.remove('active'));}
  requestDraw();
});
canvas.addEventListener('pointerup',e=>{
  if(drag&&Math.hypot(e.clientX-drag.firstX,e.clientY-drag.firstY)<4){
    const r=canvas.getBoundingClientRect(),x=e.clientX-r.left,y=e.clientY-r.top;
    const badge=labelHits.find(h=>x>=h.x&&x<=h.x+h.w&&y>=h.y&&y<=h.y+h.h);
    if(badge)select(badge.i);
    else{const near=hits.filter(h=>Math.hypot(h.x-x,h.y-y)<16);if(near.length){const current=near.findIndex(h=>h.i===selected);select(near[(current+1)%near.length].i);}}
  }
  drag=null;
});
canvas.addEventListener('pointercancel',()=>{drag=null;});
canvas.addEventListener('lostpointercapture',()=>{drag=null;});
canvas.addEventListener('contextmenu',e=>e.preventDefault());
canvas.addEventListener('wheel',e=>{e.preventDefault();camera.scale=Math.max(12,Math.min(50000,camera.scale*Math.exp(-e.deltaY*.001)));requestDraw();},{passive:false});
$('units').addEventListener('click',e=>{if(!e.target.dataset.unit)return;stopSweep();unit=e.target.dataset.unit;document.querySelectorAll('#units button').forEach(b=>b.classList.toggle('active',b.dataset.unit===unit));buildControls();update();});
$('views').addEventListener('click',e=>{const view=e.target.dataset.view;if(!view)return;const p={iso:[-1.05,.30],front:[-Math.PI/2,0],side:[0,0],top:[-Math.PI/2,Math.PI/2]}[view];[camera.yaw,camera.pitch]=p;document.querySelectorAll('#views button').forEach(b=>b.classList.toggle('active',b.dataset.view===view));fit();});
$('fit').addEventListener('click',fit);
$('zero').addEventListener('click',()=>{stopSweep();q=q.map(()=>0);update();fit();});
$('home').disabled=!model.home;
$('home').addEventListener('click',()=>{stopSweep();q=model.home.slice();update();fit();});
$('reset').addEventListener('click',()=>{stopSweep();model=clone(original);buildControls();update();fit();});
['all-frames','axes','labels','ghost','frame-size'].forEach(id=>$(id).addEventListener('input',requestDraw));
$('sweep').addEventListener('click',()=>{
  if(sweeping){stopSweep();return;}sweeping=true;sweepBase=q[selected];sweepStart=performance.now();$('sweep').textContent='■ Stop sweep';$('sweep').classList.add('active');
  const generation=++sweepGeneration;
  function animate(now){if(!sweeping||generation!==sweepGeneration)return;q[selected]=sweepBase+Math.PI/6*Math.sin((now-sweepStart)/1500);update();requestAnimationFrame(animate);}requestAnimationFrame(animate);
});
$('download').addEventListener('click',()=>{const url=URL.createObjectURL(new Blob([exportedConfig()],{type:'text/plain'}));const a=document.createElement('a');a.href=url;a.download=model.profile+'.preview.conf';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});
$('copy').addEventListener('click',async()=>{const text=$('config-preview').value;try{await navigator.clipboard.writeText(text);$('copy').textContent='Copied';}catch(e){$('config-preview').focus();$('config-preview').select();$('copy').textContent='Selected — press ⌘C / Ctrl+C';}setTimeout(()=>{$('copy').textContent='Copy kinematics block';},2500);});
$('profile-name').textContent=model.profile.toUpperCase();
document.title=model.profile.toUpperCase()+' · DH Explorer';
$('source-name').textContent=model.path.split('/').slice(-2).join('/');$('source-name').title=model.path;
$('footer-source').textContent=model.path;
$('passive-note').textContent=model.passive===null?'Joint angles preview a pose; this viewer has no robot connection.':`J${model.passive+1} is passive. Its slider previews a measured angle; this viewer has no robot connection.`;
$('export-note').textContent='Downloads preserve comments, calibration flags, and relative includes. Save the .conf beside '+model.path.split('/').pop()+' so its includes resolve. Joint pose is preview only.';
buildControls();update();resize();fit();new ResizeObserver(resize).observe($('viewport'));
// Small inspectable surface for reproducible FK and UI verification.
window.dhViewer={get model(){return clone(model);},get q(){return q.slice();},get result(){return clone(result);},select,exportedConfig,kinematicsText};
document.body.dataset.ready='true';
