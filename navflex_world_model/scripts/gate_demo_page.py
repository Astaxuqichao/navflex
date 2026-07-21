#!/usr/bin/env python3
"""Render the gate demo as one self-contained HTML page.

Reads only what the gate actually returned (the JSON files gate_demo_run.py
wrote) plus the rollout mp4s it produced. Nothing here is transcribed by hand,
so the page cannot drift away from the run it claims to show.

Images are inlined as data URIs: the page must survive being opened from
anywhere, and a Claude artifact's CSP blocks every external host anyway.

    conda activate lingbot_wm            # needs opencv
    python3 scripts/gate_demo_page.py \
        --result /tmp/gate_demo/blocked.json --result /tmp/gate_demo/open.json \
        --out /tmp/gate_demo/gate_evidence.html
"""

import argparse
import base64
import json
import pathlib
import sys

import cv2
import numpy as np

METRES_PER_FRAME = 0.188
CRITIC_FRAMES = 6


def even_sample(count, pick):
    """The frames the critic saw. Mirrors navflex_wm.critic.even_sample."""
    if pick <= 0 or count <= 0:
        return []
    if count <= pick:
        return list(range(count))
    if pick == 1:
        return [count - 1]
    step = (count - 1) / (pick - 1)
    return [round(i * step) for i in range(pick)]


def read_frames(mp4):
    cap = cv2.VideoCapture(str(mp4))
    frames = []
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frames.append(frame)
    cap.release()
    if not frames:
        raise SystemExit(f'no frames decoded from {mp4}')
    return frames


def data_uri(image, width=380):
    height = int(image.shape[0] * width / image.shape[1])
    small = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        raise SystemExit('jpeg encode failed')
    return 'data:image/jpeg;base64,' + base64.b64encode(buf.tobytes()).decode()


def esc(text):
    return (str(text).replace('&', '&amp;').replace('<', '&lt;')
            .replace('>', '&gt;').replace('"', '&quot;'))


def strip_html(record, frames):
    judged = record.get('frames_judged') or len(frames)
    cells = ''
    for i in even_sample(judged, CRITIC_FRAMES):
        seed = i == 0
        badge = '<span class="badge">真实种子帧</span>' if seed else ''
        cells += (f'<figure class="shot{" seed" if seed else ""}">'
                  f'<div class="imgwrap"><img src="{data_uri(frames[i])}" '
                  f'alt="第 {i} 帧">{badge}</div>'
                  f'<figcaption><span class="fnum">帧 {i:02d}</span>'
                  f'<span class="fdist">{i * METRES_PER_FRAME:.2f} m</span>'
                  f'</figcaption></figure>')
    tear = record.get('tear_frame') or 0
    if tear and tear + 5 < len(frames):
        cells += (f'<figure class="shot torn"><div class="imgwrap">'
                  f'<img src="{data_uri(frames[tear + 5])}" alt="撕裂后的幻觉帧">'
                  f'<span class="badge warn">已丢弃</span></div>'
                  f'<figcaption><span class="fnum">帧 {tear + 5}</span>'
                  f'<span class="fdist">幻觉</span></figcaption></figure>')
    return cells


def card_html(record, frames):
    verdict = record['verdict']
    kind = verdict if verdict in ('approve', 'reject') else 'unsure'
    label = {'approve': '放行 APPROVE', 'reject': '拦截 REJECT'}.get(
        verdict, f'待确认 {verdict.upper()}')
    tear = record.get('tear_frame') or 0
    if tear:
        note = (f'<p class="tear">世界模型在第 {tear} 帧(~{record["tear_m"]:.2f} m)'
                f'失去连贯性。闸门只把撕裂之前的 {record["frames_judged"]} 帧交给 critic,'
                f'撕裂之后的画面是模型编造的,不是证据。</p>')
    else:
        note = (f'<p class="tear ok">全程连贯,'
                f'{record.get("frames_rendered", len(frames))} 帧全部可用。</p>')
    hazards = record.get('critic', {}).get('hazards') or []
    hz = ''.join(f'<li>{esc(h)}</li>' for h in hazards) or '<li class="none">未报告</li>'
    conf = (f'置信度 {record["confidence"]:.2f}'
            if record.get('critic', {}).get('confidence_reported', True)
            else '置信度未报告')
    clearance = record.get('clearance_m')
    eyebrow = (f'深度图实测 · 正前方净空 {clearance:.2f} m' if clearance
               else f'场景 {esc(record["scenario"])}')
    return f'''<section class="row {kind}">
      <div class="rowhead">
        <div>
          <p class="eyebrow">{esc(eyebrow)}</p>
          <h3>{esc(record.get('title', record['scenario']))}</h3>
        </div>
        <p class="stamp">{label}<span class="conf">{esc(conf)}</span></p>
      </div>
      <div class="strip">{strip_html(record, frames)}</div>
      {note}
      <p class="reason">{esc(record['reason'])}</p>
      <p class="eyebrow">hazards</p>
      <ul class="hz">{hz}</ul>
    </section>'''


STYLE = '''<style>
:root{
  --ground:#F6F7F7; --panel:#FFF; --ink:#12191B; --muted:#59686C; --line:#DCE1E1;
  --approve:#1B7A4B; --approve-bg:#E6F2EB;
  --reject:#AF362F; --reject-bg:#FAE9E7;
  --signal:#8C6210;
}
@media (prefers-color-scheme: dark){:root{
  --ground:#0D1214; --panel:#151C1E; --ink:#E5EAEA; --muted:#94A2A5; --line:#253134;
  --approve:#57C089; --approve-bg:#112620;
  --reject:#E67C71; --reject-bg:#2B1715;
  --signal:#D2A244;
}}
:root[data-theme="dark"]{
  --ground:#0D1214; --panel:#151C1E; --ink:#E5EAEA; --muted:#94A2A5; --line:#253134;
  --approve:#57C089; --approve-bg:#112620;
  --reject:#E67C71; --reject-bg:#2B1715;
  --signal:#D2A244;
}
:root[data-theme="light"]{
  --ground:#F6F7F7; --panel:#FFF; --ink:#12191B; --muted:#59686C; --line:#DCE1E1;
  --approve:#1B7A4B; --approve-bg:#E6F2EB;
  --reject:#AF362F; --reject-bg:#FAE9E7;
  --signal:#8C6210;
}
*{box-sizing:border-box}
body{margin:0;background:var(--ground);color:var(--ink);line-height:1.65;font-size:16px;
  font-family:system-ui,-apple-system,"Segoe UI","Noto Sans CJK SC","PingFang SC",sans-serif}
.wrap{max-width:1120px;margin:0 auto;padding:48px 24px 80px;display:flex;flex-direction:column;gap:34px}
.mono{font-family:ui-monospace,Menlo,Consolas,monospace}
.eyebrow{font-family:ui-monospace,Menlo,monospace;text-transform:uppercase;letter-spacing:.13em;
  font-size:11px;color:var(--muted);margin:0}
h1{font-size:clamp(26px,3.3vw,36px);line-height:1.2;margin:6px 0 0;text-wrap:balance;letter-spacing:-.01em}
h3{font-size:19px;margin:2px 0 0;letter-spacing:-.01em}
.lede{margin:14px 0 0;max-width:66ch;color:var(--muted);font-size:17px}
.lede strong{color:var(--ink);font-weight:600}
.row{border:1px solid var(--line);border-top-width:3px;background:var(--panel);border-radius:2px;
  padding:20px 22px;display:flex;flex-direction:column;gap:14px}
.row.approve{border-top-color:var(--approve)}
.row.reject{border-top-color:var(--reject)}
.row.unsure{border-top-color:var(--signal)}
.rowhead{display:flex;justify-content:space-between;align-items:flex-start;gap:16px;flex-wrap:wrap}
.stamp{margin:0;font-family:ui-monospace,Menlo,monospace;font-size:13px;letter-spacing:.08em;
  padding:6px 12px;border-radius:2px;display:inline-flex;gap:12px;align-items:baseline}
.approve .stamp{background:var(--approve-bg);color:var(--approve)}
.reject .stamp{background:var(--reject-bg);color:var(--reject)}
.unsure .stamp{background:var(--ground);color:var(--signal);border:1px solid var(--signal)}
.conf{font-size:11px;opacity:.85;font-variant-numeric:tabular-nums}
.strip{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:8px}
.shot{margin:0;background:var(--ground);border:1px solid var(--line);border-radius:2px;overflow:hidden}
.shot .imgwrap{position:relative;line-height:0}
.shot img{display:block;width:100%;height:auto}
.shot.seed{border-color:var(--signal)}
.shot.torn{border-color:var(--reject);opacity:.55}
.shot.torn img{filter:grayscale(.5)}
.badge{position:absolute;left:5px;top:5px;font-family:ui-monospace,Menlo,monospace;font-size:10px;
  letter-spacing:.05em;padding:3px 6px;border-radius:2px;line-height:1.4;background:var(--signal);color:var(--panel)}
.badge.warn{background:var(--reject);color:var(--panel)}
.shot figcaption{display:flex;justify-content:space-between;padding:6px 8px;border-top:1px solid var(--line);
  font-family:ui-monospace,Menlo,monospace;font-size:11px;font-variant-numeric:tabular-nums}
.fdist{color:var(--muted)}
.tear{margin:0;font-size:14px;color:var(--reject);border-left:3px solid var(--reject);
  padding-left:12px;max-width:78ch}
.tear.ok{color:var(--muted);border-left-color:var(--line)}
.reason{margin:0;font-size:15px;max-width:78ch}
.hz{margin:0;padding:0;list-style:none;display:flex;flex-wrap:wrap;gap:6px}
.hz li{font-family:ui-monospace,Menlo,monospace;font-size:12px;padding:3px 8px;
  border:1px solid var(--line);color:var(--muted);border-radius:2px}
.hz li.none{opacity:.6}
.claim{border-left:3px solid var(--signal);background:var(--panel);border:1px solid var(--line);
  border-left:3px solid var(--signal);padding:18px 22px;display:flex;flex-direction:column;gap:8px}
.claim p{margin:0;max-width:76ch}
.claim strong{color:var(--signal)}
footer{border-top:1px solid var(--line);padding-top:20px;color:var(--muted);font-size:14px}
footer p{margin:0 0 6px;max-width:76ch}
</style>'''


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--result', action='append', required=True, type=pathlib.Path,
                    help='a JSON file from gate_demo_run.py; repeat for each scenario')
    ap.add_argument('--out', required=True, type=pathlib.Path)
    ap.add_argument('--title', default='执行前安全闸门 · 它真的会拦住障碍物吗')
    ap.add_argument('--seed-meta', type=pathlib.Path,
                    help='seed_meta.json from gate_demo_seeds.py, for the clearances')
    args = ap.parse_args()

    seeds = json.loads(args.seed_meta.read_text()) if args.seed_meta else {}

    rows = ''
    instructions = set()
    for path in args.result:
        record = json.loads(path.read_text())
        if not record.get('rollout_uri'):
            print(f'跳过 {path.name}: 没有 rollout('
                  f'verdict={record["verdict"]} — 闸门短路了,没生成画面)', file=sys.stderr)
            continue
        mp4 = pathlib.Path(record['rollout_uri'])
        if not mp4.is_file():
            raise SystemExit(f'{mp4} 不存在。页面必须在 sidecar 写出 mp4 的那台机器上生成。')
        meta = seeds.get(record['scenario'], {})
        record['clearance_m'] = meta.get('clearance_m')
        record['title'] = {'blocked': '书柜挡在正前方', 'open': '前方开阔'}.get(
            record['scenario'], record['scenario'])
        instructions.add(record['instruction'])
        rows += card_html(record, read_frames(mp4))

    if not rows:
        raise SystemExit('没有任何一条结果带 rollout,页面无从生成')

    same = (f'两次调用的指令、计划、随机种子完全一致——'
            f'<span class="mono">「{esc(next(iter(instructions)))}」</span>——'
            f'<strong>只有机器人面前的世界不同</strong>。'
            if len(instructions) == 1 else
            '每一行是一次独立调用。')

    page = f'''<title>{esc(args.title)}</title>
{STYLE}
<div class="wrap">
  <header>
    <p class="eyebrow">navflex · 第五层 · 执行前安全闸门</p>
    <h1>{esc(args.title)}</h1>
    <p class="lede">闸门在把动作下发给底盘之前,先用世界模型<strong>想象一遍执行过程</strong>,
      再让 VLM critic 判断这段想象是否安全。{same}
      种子帧由数据集自带的深度图挑出,不是靠肉眼选的。</p>
  </header>

  {rows}

  <div class="claim">
    <p class="eyebrow">闸门必须先怀疑自己的想象</p>
    <p>世界模型<strong>没有碰撞的概念</strong>。撞进书柜之后,它无话可画,于是编造了一个
      并不存在的机器人填满画面。第一次跑这个实验时 critic 确实拦下了——但它引用的理由是
      <strong>那个幻觉</strong>,不是柜子。<strong>对的判决,错的证据。</strong>
      换一次幻觉画成一条通畅的走廊,它就会放行。</p>
    <p>所以闸门先量自己的 rollout:逐帧差一旦超过此前中位数的 3 倍,就判定想象已经崩了,
      只把撕裂之前的帧交给 critic。而且<strong>想象若在到达目标前就崩了,approve 一律降级为
      needs_confirmation</strong>——critic 可以诚实地批准它看到的那一段,那不等于整条路安全。</p>
    <p>指令<strong>只发给 critic,不发给世界模型</strong>。否则你告诉扩散模型"任务是上楼梯",
      它就有动机把楼梯画出来,闸门变成回音壁。</p>
  </div>

  <footer>
    <p>世界模型 lingbot-world-v2-14b-causal-fast(18.5B DiT,单张 RTX 5090,group offloading)。
      每帧对应 {METRES_PER_FRAME} m(用里程计标定),29 帧的 rollout 描绘 5.26 m。
      种子帧与深度真值取自 MATRiX 数据集。</p>
    <p>闸门在拿不到证据时一律 <span class="mono">fail closed</span>:相机无画面、sidecar 掉线、
      计划超出可连贯想象的 5.26 m、rollout 撕裂过早,都会返回 needs_confirmation,而不是放行。</p>
    <p><strong>尚未验证的:</strong>样本量很小;聚合器会把同一模型路由到不同供应商;
      而且只测过误放行,<strong>没测吞吐率</strong>——闸门会不会误拦大量正常任务,一次都没量过。</p>
  </footer>
</div>
'''
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(page, encoding='utf-8')
    print(f'wrote {args.out}  ({args.out.stat().st_size / 1024:.0f} KB)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
