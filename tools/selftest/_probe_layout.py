# -*- coding: utf-8 -*-
"""
浏览器级布局自检：在真实浏览器（headless Edge）中派发 pointer 事件模拟拖拽分隔条，
校验：初始尺寸、1:1 跟手、双击复位、localStorage 持久化、无横向溢出。

用法： python _probe_layout.py
依赖： Edge/Chrome 可执行文件（见下方 EDGE 常量，可用环境变量 BROWSER_EXE 覆盖）
"""
import pathlib, re, subprocess, sys, os

BASE = pathlib.Path(__file__).resolve().parent
HTML = BASE.parent / "serial-studio.html"
EDGE = os.environ.get("BROWSER_EXE") or r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"

html = HTML.read_text(encoding="utf-8")
probe = """
<script>
window.addEventListener('load', () => setTimeout(() => {
  const out = [];
  const P = (name, cond, info) => out.push((cond ? 'PASS  ' : 'FAIL  ') + name + (info ? '  [' + info + ']' : ''));
  const cs = () => getComputedStyle(document.documentElement);
  const midCol = document.querySelector('main > .col.mid');
  const leftCol = document.querySelector('main > .col');
  const rightCol = document.querySelectorAll('main > .col')[2];
  const chart = document.getElementById('chartCard');
  const spL = document.getElementById('spL'), spR = document.getElementById('spR'), spLog = document.getElementById('spLog');
  const fire = (el, type, x, y, id) => el.dispatchEvent(new PointerEvent(type,
    { clientX: x, clientY: y, pointerId: id || 1, bubbles: true, cancelable: true }));
  try { localStorage.removeItem('serial-studio-layout'); } catch {}

  P('三个分隔条都存在', !!spL && !!spR && !!spLog);
  P('初始 --wL = 326px', Math.abs(parseFloat(cs().getPropertyValue('--wL')) - 326) <= 1, cs().getPropertyValue('--wL').trim());
  P('初始 --chartH = 250px', Math.abs(parseFloat(midCol.style.getPropertyValue('--chartH')) - 250) <= 1,
    midCol.style.getPropertyValue('--chartH'));

  const w0 = leftCol.getBoundingClientRect().width, x0 = spL.getBoundingClientRect().left + 3;
  fire(spL, 'pointerdown', x0, 500); fire(spL, 'pointermove', x0 + 120, 500); fire(spL, 'pointerup', x0 + 120, 500);
  const w1 = leftCol.getBoundingClientRect().width;
  P('左分隔条右移 120px -> 左栏 +120px', Math.abs((w1 - w0) - 120) <= 3, w0.toFixed(0) + ' -> ' + w1.toFixed(0));

  const r0 = rightCol.getBoundingClientRect().width, rx = spR.getBoundingClientRect().left + 3;
  fire(spR, 'pointerdown', rx, 500, 2); fire(spR, 'pointermove', rx - 80, 500, 2); fire(spR, 'pointerup', rx - 80, 500, 2);
  const r1 = rightCol.getBoundingClientRect().width;
  P('右分隔条左移 80px -> 右栏 +80px', Math.abs((r1 - r0) - 80) <= 3, r0.toFixed(0) + ' -> ' + r1.toFixed(0));

  const c0 = chart.getBoundingClientRect().height, sy = spLog.getBoundingClientRect().top + 3;
  fire(spLog, 'pointerdown', 700, sy, 3); fire(spLog, 'pointermove', 700, sy - 100, 3); fire(spLog, 'pointerup', 700, sy - 100, 3);
  const c1 = chart.getBoundingClientRect().height;
  P('日志分隔条上移 100px -> 波形 +100px', Math.abs((c1 - c0) - 100) <= 3, c0.toFixed(0) + ' -> ' + c1.toFixed(0));

  let saved = null;
  try { saved = JSON.parse(localStorage.getItem('serial-studio-layout')); } catch {}
  P('尺寸写入 localStorage', saved && Math.abs(saved.wL - 446) < 40 && Math.abs(saved.chartH - 350) < 40, JSON.stringify(saved));

  fire(spL, 'dblclick', x0, 500, 4);
  P('双击左分隔条复位 326px', Math.abs(parseFloat(cs().getPropertyValue('--wL')) - 326) <= 1, cs().getPropertyValue('--wL').trim());
  fire(spLog, 'dblclick', 700, sy, 5);
  P('双击日志分隔条复位 250px', Math.abs(parseFloat(midCol.style.getPropertyValue('--chartH')) - 250) <= 1,
    midCol.style.getPropertyValue('--chartH'));

  P('无横向溢出', document.documentElement.scrollWidth <= window.innerWidth + 1,
    document.documentElement.scrollWidth + '/' + window.innerWidth);

  const pre = document.createElement('pre');
  pre.id = 'probeOut';
  pre.textContent = out.join('\\n') + '\\n结果: ' + out.filter(x => x.indexOf('PASS') === 0).length + ' / ' + out.length + ' 项通过';
  document.body.appendChild(pre);
}, 700));
</script>
</body>"""

test = BASE / "_probe_layout.html"
test.write_text(html.replace("</body>", probe, 1), encoding="utf-8")

r = subprocess.run([EDGE, "--headless=new", "--disable-gpu", "--window-size=1600,1000",
                    "--virtual-time-budget=6000", "--dump-dom",
                    "file:///" + str(test).replace("\\", "/")],
                   capture_output=True, encoding="utf-8", errors="replace")
sys.stdout.reconfigure(encoding="utf-8", errors="replace")
m = re.search(r'<pre id="probeOut">(.*?)</pre>', r.stdout or "", re.S)
if m:
    text = m.group(1).replace("&lt;", "<").replace("&gt;", ">").replace("&amp;", "&")
    print(text)
    sys.exit(0 if "FAIL" not in text else 1)
print("未取到探针输出，DOM 长度", len(r.stdout or ""))
print((r.stderr or "")[:1200])
sys.exit(1)
