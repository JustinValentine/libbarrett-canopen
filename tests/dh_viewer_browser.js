/* Runs inside the generated offline page; no browser packages or server. */
(async () => {
  const failures=[],checks=[];
  const check=(name,fn)=>{try{fn();checks.push(name);}catch(e){failures.push(name+': '+e.message);}};
  const assert=(value,message='assertion failed')=>{if(!value)throw new Error(message);};
  const close=(actual,expected,tolerance=1e-10)=>{
    if(Array.isArray(expected)){assert(actual.length===expected.length);expected.forEach((v,i)=>close(actual[i],v,tolerance));}
    else assert(Math.abs(actual-expected)<=tolerance,`${actual} != ${expected}`);
  };
  const input=(id,value)=>{const e=document.getElementById(id);e.value=value;e.dispatchEvent(new Event('input',{bubbles:true}));return e;};
  const click=id=>document.getElementById(id).click();
  const viewer=window.dhViewer;
  check('page initializes without errors',()=>{assert(viewer);assert(document.body.dataset.ready==='true');assert(!window.testErrors.length,window.testErrors.join('\n'));});
  if(viewer){
    const initial=viewer.model,n=initial.moving.length;
    check('zero joint pose retains configured fixed offsets',()=>{
      close(viewer.q,Array(n).fill(0));
      initial.moving.forEach((row,i)=>close(Number(document.getElementById(`dh-${i}-theta_pi`).value),row.theta_pi??0,.000051));
    });
    check('unedited export preserves the complete source',()=>assert(viewer.exportedConfig()===initial.source));
    check('all controls match the loaded profile',()=>{
      assert(document.querySelectorAll('.joint').length===n);
      assert(document.querySelectorAll('#dh-rows tr').length===n+1);
      assert(document.querySelectorAll('#dh-rows input[data-key="theta_pi"]').length===n+1);
      assert(document.getElementById('passive-note').textContent.includes('passive')===(initial.passive!==null));
    });
    check('joint input, units, and slider remain consistent',()=>{
      input('q-number-0',90);close(viewer.q[0],Math.PI/2);
      close(Number(document.getElementById('dh-0-theta_pi').value),initial.moving[0].theta_pi);
      document.querySelector('[data-unit="rad"]').click();
      close(Number(document.getElementById('q-number-0').value),Math.PI/2,.00051);
      input('q-slider-0',.75);close(viewer.q[0],.75,.001);
      document.querySelector('[data-unit="deg"]').click();
      assert(viewer.exportedConfig()===initial.source,'preview pose changed the config');
    });
    check('theta_pi edits a fixed offset independently of the joint pose',()=>{
      const before=viewer.q.slice();
      input('dh-0-theta_pi',-.5);close(viewer.q,before);
      close(viewer.model.moving[0].theta_pi,-.5);
      close(viewer.result.tool,DHMath.fk(viewer.model,before).tool);
      assert(/theta_pi\s*=\s*-0\.5\s*;/.test(viewer.exportedConfig()),'moving offset was not exported');
      window.testOffsetExport=viewer.exportedConfig();
      document.querySelector('[data-unit="rad"]').click();
      close(Number(document.getElementById('dh-0-theta_pi').value),-.5);
      document.querySelector('[data-unit="deg"]').click();
      click('zero');close(viewer.q,Array(n).fill(0));
      close(Number(document.getElementById('dh-0-theta_pi').value),-.5);
      click('reset');assert(viewer.exportedConfig()===initial.source);
    });
    check('home and zero poses',()=>{
      if(initial.home){click('home');close(viewer.q,initial.home);}
      initial.moving.forEach((r,i)=>close(Number(document.getElementById(`dh-${i}-theta_pi`).value),r.theta_pi));
      click('zero');close(viewer.q,Array(n).fill(0));
      initial.moving.forEach((r,i)=>close(Number(document.getElementById(`dh-${i}-theta_pi`).value),r.theta_pi));
    });
    check('editing preserves source comments and unrelated fields',()=>{
      input('dh-0-a',123.456789012345);close(viewer.model.moving[0].a,.123456789012345);
      const f=initial.fields.find(f=>f.row===0&&f.key==='a');
      const expected=initial.source.slice(0,f.span[0])+String(viewer.model.moving[0].a)+initial.source.slice(f.span[1]);
      assert(viewer.exportedConfig()===expected,'export modified additional content');
      assert(document.getElementById('model-state').textContent==='Edited');
      window.testExport=viewer.exportedConfig();
    });
    check('invalid input cannot be exported and recovers on blur',()=>{
      for(const id of ['dh-0-a','q-number-0','dh-0-theta_pi',`dh-${n}-theta_pi`]){
        const e=input(id,'');assert(e.getAttribute('aria-invalid')==='true');
        assert(document.getElementById('download').disabled);
        assert(document.getElementById('copy').disabled);
        e.dispatchEvent(new Event('blur'));
        assert(e.getAttribute('aria-invalid')==='false');
        assert(!document.getElementById('download').disabled);
      }
      input('dh-0-d','1e308');assert(document.getElementById('download').disabled);
      document.getElementById('dh-0-d').dispatchEvent(new Event('blur'));
    });
    check('tool theta selects tool and disables sweep',()=>{
      input(`dh-${n}-theta_pi`,.5);close(viewer.model.tool.theta_pi,.5);
      assert(/toolplate\s*[:=]\s*\{[^}]*theta_pi\s*=\s*0\.5\s*;/.test(viewer.exportedConfig()),'tool theta was not exported');
      assert(document.getElementById('sweep').disabled);
      assert(document.getElementById('matrix-title').textContent==='Toolplate → world transform');
    });
    check('reset restores exact original source',()=>{
      click('reset');assert(viewer.exportedConfig()===initial.source);
      assert(document.getElementById('model-state').textContent==='Original');
    });
    check('stopping and restarting a sweep leaves one animation',()=>{
      viewer.select(0);
      const real=window.requestAnimationFrame,pending=[];
      window.requestAnimationFrame=fn=>{pending.push(fn);return 1;};
      try {
        click('sweep');click('sweep');click('sweep');
        const initialCallbacks=pending.splice(0);
        initialCallbacks.forEach(fn=>fn(performance.now()+100));
        close(Number(document.getElementById('dh-0-theta_pi').value),initial.moving[0].theta_pi);
        assert(pending.filter(fn=>fn.name==='animate').length===1,'multiple sweep loops');
        click('zero');const before=viewer.q;
        pending.splice(0).forEach(fn=>fn(performance.now()+200));close(viewer.q,before);
      }finally{window.requestAnimationFrame=real;}
    });
    check('analytical planar two-link pose',()=>{
      const flat={world:DHMath.identity(),moving:[{alpha_pi:0,a:1,d:0},{alpha_pi:0,a:1,d:0}],tool:{alpha_pi:0,theta_pi:0,a:0,d:0}};
      const fk=DHMath.fk(flat,[Math.PI/2,-Math.PI/2]);
      close(DHMath.point(fk.tool),[1,1,0]);close(fk.joints[1].origin,[0,1,0]);close(fk.joints[1].axis,[0,0,1]);
      flat.moving[0].theta_pi=.5;flat.moving[1].theta_pi=-.5;
      close(DHMath.fk(flat,[0,0]).tool,fk.tool);
    });
    if(window.nativeCases.length)check('production C forward kinematics and rotation axes',()=>{
      for(const {model,q,expected} of window.nativeCases){
        const fk=DHMath.fk(model,q);close(fk.frames,expected.frames);close(fk.tool,expected.tool);
        fk.joints.forEach((joint,i)=>{close(joint.origin,expected.frames[i].slice(0,3).map(row=>row[3]));close(joint.axis,expected.frames[i].slice(0,3).map(row=>row[2]));});
      }
    });
    check('download contains the edited configuration',()=>{
      const create=URL.createObjectURL,revoke=URL.revokeObjectURL,anchorClick=HTMLAnchorElement.prototype.click;
      let blob,filename;
      URL.createObjectURL=value=>{blob=value;return 'blob:test';};URL.revokeObjectURL=()=>{};
      HTMLAnchorElement.prototype.click=function(){filename=this.download;};
      try{click('download');assert(blob instanceof Blob);assert(filename===initial.profile+'.preview.conf');window.testDownload=blob;}
      finally{URL.createObjectURL=create;URL.revokeObjectURL=revoke;HTMLAnchorElement.prototype.click=anchorClick;}
    });
    if(window.testDownload){const text=await window.testDownload.text();check('download text round-trips',()=>assert(text===viewer.exportedConfig()));}
    check('view and overlay controls render',()=>{
      for(const view of ['front','side','top','iso'])document.querySelector(`[data-view="${view}"]`).click();
      for(const id of ['all-frames','axes','labels','ghost'])click(id);
      input('frame-size',120);click('fit');
      for(const id of ['all-frames','axes','labels','ghost'])click(id);
      input('frame-size',85);if(initial.home)click('home');
    });
    // Let the requested canvas redraw and ResizeObserver finish before reporting.
    await new Promise(resolve=>setTimeout(resolve,100));
    window.flushTestFrame();window.flushTestFrame();
    check('canvas drew visible geometry without runtime errors',()=>{
      const c=document.getElementById('scene');assert(c.width>0&&c.height>0,'canvas has no size');
      assert(c.getContext('2d').getImageData(0,0,c.width,c.height).data.some(v=>v!==0),'canvas has no pixels');
      assert(!window.testErrors.length,window.testErrors.join('\n'));
      assert(document.documentElement.scrollWidth<=innerWidth,'horizontal page overflow');
    });
  }
  const report=document.createElement('script');report.id='test-report';report.type='application/json';
  report.textContent=JSON.stringify({checks,failures,exported:window.testExport,offsetExport:window.testOffsetExport,nativeCases:window.nativeCases.length}).replace(/</g,'\\u003c');document.body.append(report);
  // Screenshots can trigger a final viewport resize; return to Chrome's clock
  // so its ResizeObserver redraw is painted normally after assertions finish.
  window.requestAnimationFrame=window.nativeAnimationFrame;window.flushTestFrame();
})().catch(e=>{document.body.dataset.testFatal=e.stack;});
