# -*- coding: utf-8 -*-
"""
启动恢复自检：预置 localStorage 中的布局尺寸后打开页面，
校验记忆尺寸在启动时即被采用（无跳变），且运行时状态与呈现一致。

用法： python _probe_restore.py
"""
import pathlib, re, subprocess, sys, os, shutil

HERE = pathlib.Path(__file__).resolve().parent
EDGE = os.environ.get("BROWSER_EXE") or r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"
SAVED = {"wL": 420, "wR": 300, "chartH": 320}

# 1) 中转页：写入 localStorage 后跳转到主页面（同目录 => file:// 同源，可共享存储）
relay = HERE / "_probe_restore.html"
relay.write_text("""<!DOCTYPE html><meta charset="utf-8">
<script>
try { localStorage.setItem('serial-studio-layout', JSON.stringify(%s)); } catch (e) {}
location.replace('../serial-studio.html');
</script>
""" % __import__("json").dumps(SAVED), encoding="utf-8")

# 2) 带探针的主页面副本
probe = """
<script>
window.addEventListener('load', () => setTimeout(() => {
  const mid = document.querySelector('main > .col.mid');
  const chart = document.getElementById('chartCard');
  const cols = document.querySelectorAll('main > .col');
  const out = [];
  const P = (n, c, i) => out.push((c ? 'PASS  ' : 'FAIL  ') + n + (i ? '  [' + i + ']' : ''));
  const near = (a, b) => Math.abs(a - b) <= 2;
  P('恢复左栏宽度 ' + %d + 'px', near(cols[0].getBoundingClientRect().width, %d), cols[0].getBoundingClientRect().width.toFixed(1));
  P('恢复右栏宽度 ' + %d + 'px', near(cols[2].getBoundingClientRect().width, %d), cols[2].getBoundingClientRect().width.toFixed(1));
  P('恢复波形高度 ' + %d + 'px', near(chart.getBoundingClientRect().height, %d), chart.getBoundingClientRect().height.toFixed(1));
  P('--chartH 与呈现一致', near(parseFloat(mid.style.getPropertyValue('--chartH')), chart.getBoundingClientRect().height),
    mid.style.getPropertyValue('--chartH'));
  P('无横向溢出', document.documentElement.scrollWidth <= window.innerWidth + 1,
    document.documentElement.scrollWidth + '/' + window.innerWidth);
  const pre = document.createElement('pre'); pre.id = 'probeOut';
  pre.textContent = out.join('\\n') + '\\n结果: ' + out.filter(x => x.indexOf('PASS') === 0).length + ' / ' + out.length + ' 项通过';
  document.body.appendChild(pre);
}, 700));
</script>
</body>""" % (SAVED["wL"], SAVED["wL"], SAVED["wR"], SAVED["wR"], SAVED["chartH"], SAVED["chartH"])

run = HERE / "_probe_restore_run.html"
run.write_text((HERE.parent / "serial-studio.html").read_text(encoding="utf-8").replace("</body>", probe, 1), encoding="utf-8")

# 3) 同一个 profile 跑两趟：第一趟写存储，第二趟验证启动恢复
profile = HERE / "_edge_profile"
if profile.exists():
    shutil.rmtree(profile, ignore_errors=True)
common = ["--headless=new", "--disable-gpu", f"--user-data-dir={profile}", "--window-size=1600,1000"]
subprocess.run([EDGE, *common, "--virtual-time-budget=3000", "--dump-dom",
                "file:///" + str(relay).replace("\\", "/")], capture_output=True, encoding="utf-8", errors="replace")
r = subprocess.run([EDGE, *common, "--virtual-time-budget=5000", "--dump-dom",
                    "file:///" + str(run).replace("\\", "/")], capture_output=True, encoding="utf-8", errors="replace")

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
m = re.search(r'<pre id="probeOut">(.*?)</pre>', r.stdout or "", re.S)
if not m:
    print("未取到探针输出；DOM 长度", len(r.stdout or ""))
    print((r.stderr or "")[:900])
    sys.exit(1)
text = m.group(1).replace("&lt;", "<").replace("&gt;", ">").replace("&amp;", "&")
print(text)
sys.exit(0 if "FAIL" not in text else 1)
