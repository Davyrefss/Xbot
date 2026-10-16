# -*- coding: utf-8 -*-
"""
视觉验证（截图）：注入一段「帧头与当前配置不匹配」的模拟数据，
确认「收到数据但切不出帧」的警告横幅被触发并正确渲染。

只做截图，不做 --dump-dom（页面有常驻定时器会让 dump 不返回）。
用法： python _probe_banner.py     产出 _probe_banner.png
"""
import pathlib, subprocess, sys, os, time

HERE = pathlib.Path(__file__).resolve().parent
EDGE = os.environ.get("BROWSER_EXE") or r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"

inject = """
<script>
window.addEventListener('load', () => setTimeout(() => {
  const S = window.__S;
  if (!S) return;
  S.connected = true; S.demo = false; S.closing = false;
  S.parserStalled = false; S.stallWarned = true;
  S.startedAt = Date.now(); S.lastRxAt = Date.now() - 3000;
  S.rxBytes = 0; S.frameCount = 0;
  const bytes = [0x00, 0x11, 0x22, 0xff];
  for (let s = 0; s < 24; s++) {              // 5A A5 帧头，帧长 15，当前配置是 AA 55 故切不出帧
    const b = [0x5a, 0xa5, 0x00, s & 0xff];
    for (let i = 0; i < 10; i++) b.push((s * 7 + i * 13) & 0xff);
    b[2] = b.length + 1;
    let sum = 0; for (const x of b) sum = (sum + x) & 0xff;
    b.push(sum);
    bytes.push(...b);
  }
  window.__feedRaw(Uint8Array.from(bytes));
  /* 不直接调 checkNoFrames：等 500ms 周期的真实看门狗触发，验证的是真实路径 */
  window.__checkNoFrames = window.__checkNoFrames;
  setTimeout(() => {
    const el = document.getElementById('logBanner');
    const dbg = document.createElement('div');
    dbg.style.cssText = 'position:fixed;left:8px;bottom:44px;z-index:999;background:#000c;color:#0f0;'
      + 'font:12px monospace;padding:6px 10px;white-space:pre;border-radius:6px';
    dbg.id = 'dbgOut';
    dbg.textContent = 'rx=' + window.__S.rxBytes + ' frames=' + window.__S.frameCount
      + ' dropped=' + window.__S.dropCount + ' stalled=' + window.__S.parserStalled
      + '\\nbanner.display="' + (el ? el.style.display : 'no-el') + '"'
      + '\\ntext="' + (el ? el.textContent.replace(/\\s+/g, ' ').slice(0, 150) : '') + '"';
    document.body.appendChild(dbg);
    document.title = 'banner:' + (el && el.style.display !== 'none' ? 'shown' : 'hidden');
  }, 1700);
}, 800));
</script>
</body>"""

html = (HERE.parent / "serial-studio.html").read_text(encoding="utf-8")
html = html.replace("  window.__rawHex = (n) => hexBytesHex(n || 512);",
                    "  /* 与 readLoop 完全一致的入口顺序：先入原始字节缓冲，再进解析器 */\n"
                    "  window.__S = S;\n"
                    "  window.__feedRaw = (b) => { S.rxBytes += b.length; S.lastRxAt = Date.now(); hexBytesFeed(b); rxIndicator(b); feedParser(b); };\n"
                    "  window.__checkNoFrames = () => checkNoFrames();\n"
                    "  window.__rawHex = (n) => hexBytesHex(n || 512);", 1)
assert "__feedRaw" in html, "钩子注入失败"
run = HERE / "_probe_banner.html"
run.write_text(html.replace("</body>", inject, 1), encoding="utf-8")

shot = HERE / "_probe_banner.png"
if shot.exists():
    shot.unlink()
r = subprocess.run([EDGE, "--headless=new", "--disable-gpu", "--hide-scrollbars",
                    "--window-size=1600,1000", "--virtual-time-budget=4500",
                    f"--screenshot={shot}", "file:///" + str(run).replace("\\", "/")],
                   capture_output=True, encoding="utf-8", errors="replace", timeout=120)
print("截图:", shot.name, "大小", shot.stat().st_size if shot.exists() else 0)
sys.exit(0 if shot.exists() and shot.stat().st_size > 0 else 1)
