/* Raw tactile presentation. Replay uses capture age; live age advances locally. */
window.LingxiTactile = (() => {
  const views = new WeakMap();
  const names = ['palm','back_of_hand','thumb','index','middle','ring','little'];
  const statusText = {fresh:'数据可用',stale:'已过期 · 下方为旧值',missing:'缺少数据',invalid:'数据格式异常',time_error:'时间异常 · 不作为当前值'};
  function update(root){
    const view=views.get(root); if(!view)return '';
    const elapsed=view.replay?0:(performance.now()-view.arrived)/1000;
    const descriptions=[];
    for(const item of view.items){
      const q=item.quality;
      const age=typeof q.age_s==='number'?q.age_s+elapsed:null;
      const status=q.status==='fresh'&&age>view.staleAfter?'stale':q.status;
      item.section.dataset.status=status;
      item.section.classList.toggle('tactile-old',status==='stale'||status==='time_error');
      item.label.textContent=`${item.side==='left'?'左手':'右手'} · ${q.synthetic?'合成数据 · ':''}${statusText[status]||'数据异常'}${age===null?'':` · ${view.replay?'采集时接收龄':'页面估计接收龄'} ${age.toFixed(3)} s`}`;
      descriptions.push(item.label.textContent);
    }
    return descriptions.join('；');
  }
  function render(root,frames,quality,replay=false){
    root.replaceChildren();
    const view={arrived:performance.now(),replay,staleAfter:quality?.stale_after_s??0.5,items:[]};
    for(const side of ['left','right']){
      const q=quality?.sides?.[side]||{status:'missing',age_s:null};
      const section=document.createElement('section');section.dataset.side=side;
      const label=document.createElement('p');label.className='tactile-state';label.setAttribute('role','status');section.append(label);
      view.items.push({side,quality:q,section,label});
      const frame=frames?.[side];
      if(frame&&['fresh','stale','time_error'].includes(q.status)){
        const detail=document.createElement('p');detail.className='tactile-detail';
        detail.textContent=`原始值峰值 ${q.peak_raw_uint8}/255 · 非零 ${q.nonzero_cells}/${q.cell_count} · 达字节上限 ${q.raw_ceiling_cells} 格${q.raw_ceiling_cells?'（不代表已确认物理饱和）':''}`;
        section.append(detail);
        const surfaces={palm:frame.palm,back_of_hand:frame.back_of_hand,...frame.fingertips};
        for(const name of names){
          const surface=surfaces[name];const wrap=document.createElement('div');wrap.className='tactile-surface';
          const title=document.createElement('div');title.textContent=`${surface.name} · peak ${Math.max(...surface.values)}`;wrap.append(title);
          const canvas=document.createElement('canvas');const [rows,cols]=surface.shape;canvas.width=cols*24;canvas.height=rows*24;
          const ctx=canvas.getContext('2d');
          surface.values.forEach((v,i)=>{const x=i%cols*24,y=Math.floor(i/cols)*24;ctx.fillStyle=`rgb(${Math.round(v*0.9)},${30+Math.round(v*0.6)},${40+Math.round(v*0.2)})`;ctx.fillRect(x,y,23,23);ctx.fillStyle=v>150?'#101417':'#fff';ctx.font='10px monospace';ctx.fillText(String(v),x+2,y+15);});
          canvas.setAttribute('aria-label',`${q.synthetic?'合成数据 ':''}${side} ${surface.name}: ${surface.values.join(', ')}`);wrap.append(canvas);section.append(wrap);
        }
      }else{
        const empty=document.createElement('p');empty.className='tactile-empty';empty.textContent=q.status==='invalid'?'无可显示的有效网格':'没有该侧有效帧；不以零值代替';section.append(empty);
      }
      root.append(section);
    }
    views.set(root,view);return update(root);
  }
  return {render,update};
})();
