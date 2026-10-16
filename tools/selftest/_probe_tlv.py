# -*- coding: utf-8 -*-
"""最终验证：真实浏览器里注入 TLV 帧，确认「最新解析结果」的 SEQ/CMD/DATA 都正确"""
import pathlib, re, subprocess, sys, os

HERE = pathlib.Path(__file__).resolve().parent
EDGE = os.environ.get("BROWSER_EXE") or r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"

inject = """
<script>
window.addEventListener('load', () => setTimeout(() => {
  const S = window.__S;
  /* 应用与「示例：TLV 帧」按钮完全相同的配置（通过真实按钮点击，确保测的是用户路径） */
  document.getElementById('btnPresetTLV').click();
  const cfg = {
    pMode: document.getElementById('pMode').value,
    lenHead: document.getElementById('lenHead').value,
    lenPos: document.getElementById('lenPos').value,
    lenKind: document.getElementById('lenKind').value,
    lenExtra: document.getElementById('lenExtra').value,
    crcAlgo: document.getElementById('crcAlgo').value,
    crcScope: document.getElementById('crcScope').value,
    fields: [...window.__fields()].map(f => f.name + '@' + f.off + '/' + f.type + '/' + f.len),
  };
  /* 三帧：LEN=0 / LEN=2 / LEN=8 */
  const frames = [
    [0xA5,0x00,0x01,0x01,0x8E],
    [0xA5,0x02,0x02,0x10,0x01,0x02,0x31],
    [0xA5,0x08,0x03,0x20,0,1,2,3,4,5,6,7,0x03],
  ];
  for (const f of frames) window.__feedRaw(Uint8Array.from(f));
  /* 直接验证 hex 与 FIELD_TYPES 行为 */
  const dbg = { hexFn: typeof window.__hexFn, hex3: String(window.__hexFn(Uint8Array.from([1,2,3]))),
    readHex: String(window.__fieldRead('hex', Uint8Array.from([9,8,7,6,5,4]), 2, 2)),
    readAscii: String(window.__fieldRead('ascii', Uint8Array.from([65,66,67,68]), 1, 2)) };
  console.log('DBG ' + JSON.stringify(dbg));
  const rows = [...document.querySelectorAll('#lastTable tbody tr')]
    .map(tr => tr.children[0].textContent.trim() + '=' + tr.children[1].textContent.trim());
  const rxLines = S.lines.filter(l => l.dir === 'rx');
  const out = {
    cfg,
    frames: rxLines.map(l => l.extra),
    最后一帧解析: rows,
    帧数: S.frameCount,
  };
  const pre = document.createElement('pre'); pre.id = 'probeOut';
  pre.style.cssText = 'position:fixed;left:8px;bottom:8px;z-index:99;background:#000d;color:#0f0;font:11px monospace;padding:8px;white-space:pre-wrap;max-width:1500px';
  out.dbg = dbg; pre.textContent = JSON.stringify(out, null, 1);
  document.body.appendChild(pre);
}, 900));
</script>
</body>"""

html = (HERE.parent / "serial-studio.html").read_text(encoding="utf-8")
html = html.replace("  window.__rawHex = (n) => hexBytesHex(n || 512);",
                    "  window.__S = S; window.__fields = () => fieldDefs; window.__hexFn = hex; window.__fieldRead = (ty, b, o, l) => FIELD_TYPES[ty].read(b, o, undefined, l);\n"
                    "  window.__feedRaw = (b) => { S.rxBytes += b.length; S.lastRxAt = Date.now(); hexBytesFeed(b); rxIndicator(b); feedParser(b); };\n"
                    "  window.__rawHex = (n) => hexBytesHex(n || 512);", 1)
assert "__feedRaw" in html and "__fields" in html
run = HERE / "_probe_tlv.html"
run.write_text(html.replace("</body>", inject, 1), encoding="utf-8")

shot = HERE / "_probe_tlv.png"
r = subprocess.run([EDGE, "--headless=new", "--disable-gpu", "--hide-scrollbars",
                    "--window-size=1600,1000", "--virtual-time-budget=3000",
                    f"--screenshot={shot}", "file:///" + str(run).replace("\\", "/")],
                   capture_output=True, encoding="utf-8", errors="replace", timeout=120)
print("截图:", shot.name, shot.stat().st_size if shot.exists() else 0)
print((r.stderr or "")[:400])
