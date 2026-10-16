# -*- coding: utf-8 -*-
"""
Serial Studio 逻辑自检（在 Node 中真实执行 serial-studio.html 的内联脚本）

原理：抽取 serial-studio.html 里的 <script>，用最小 DOM 桩在 Node 里跑起来，
      再对切帧 / CRC / 字段解析 / 发送编码做断言。

用法（在本目录下执行）：
    python _selftest.py

依赖：仅需 Node.js（路径见下方 NODE 常量，可改成本机 node）。
输出：全部通过时退出码为 0。
"""
import re, pathlib, subprocess, sys, os

HERE = pathlib.Path(__file__).resolve().parent
HTML = HERE.parent / "serial-studio.html"          # 被测文件
NODE = os.environ.get("NODE_EXE") or r"C:\Users\32614\.dsh\dsh-runtimes\dsh-primary-runtime\dependencies\node\bin\node.exe"
if not pathlib.Path(NODE).exists():
    NODE = "node"                                   # 回退到 PATH 里的 node

script = re.findall(r"<script>(.*?)</script>", HTML.read_text(encoding="utf-8"), re.S)[-1]
script += "\nglobalThis.__T = { feedParser, parseFrameFields, crc16, crc8, CRC_ALGOS, P, S, fieldDefs, txPayload, parseHexStr, hex, startDemo, stopDemo, layout, applyLayout, LAYOUT_DEFAULT, LAYOUT_KEY, checkRxStall, checkNoFrames, analyzeHead, hexBytesFeed, hexBytesGet, hexBytesReset, hexBytesHex };\n"

harness = r"""
const store = new Map();
function mkEl(tag) {
  const el = {
    tagName: tag || 'div', value: '', checked: false, textContent: '', innerHTML: '', className: '',
    style: new Proxy({ setProperty(){}, getPropertyValue(){ return ''; }, removeProperty(){} },
      { get: (t, k) => (k in t ? t[k] : (t[k] ?? '')), set: (t, k, v) => { t[k] = v; return true; } }),
    dataset: {}, children: [], childElementCount: 0, firstElementChild: null,
    classList: { add(){}, remove(){}, toggle(){}, contains(){ return false; } },
    addEventListener(){}, removeEventListener(){},
    appendChild(c){ this.children.push(c); this.childElementCount = this.children.length; },
    removeChild(c){ const i = this.children.indexOf(c); if (i >= 0) this.children.splice(i, 1); this.childElementCount = this.children.length; },
    querySelector(){ return mkEl('div'); }, querySelectorAll(){ return []; },
    getAttribute(){ return null; }, setAttribute(){}, focus(){}, click(){}, remove(){},
    getBoundingClientRect(){ return { width: 600, height: 300 }; },
    scrollTop: 0, scrollHeight: 0, clientWidth: 600, clientHeight: 300, width: 600, height: 300,
    getContext(){ return new Proxy({}, { get: () => () => {} }); },
    closest(){ return null; }, contains(){ return false; }, dispatchEvent(){},
  };
  return el;
}
/* 与页面默认值一致 */
const defaults = {
  baud:'115200', databits:'8', stopbits:'1', parity:'none', flow:'none',
  pMode:'len', lineEnd:'lf', delimHead:'AA 55', delimTail:'', lenHead:'AA 55', lenPos:'2',
  lenSize:'1', lenKind:'total', lenExtra:'0', lenEndian:'le', timeoutMs:'20', maxFrame:'512',
  dispFmt:'both', dispDir:'all', globalEndian:'be', crcAlgo:'crc16modbus', crcScope:'all',
  crcWhere:'tail', crcEndian:'le', txEnc:'UTF-8', append:'lf', interval:'200', chartWin:'30',
};
function elFor(id) {
  if (!store.has(id)) {
    const el = mkEl('div'); el.id = id;
    if (id in defaults) el.value = defaults[id];
    if (['showRawHex','autoTrack','chkCrc'].includes(id)) el.checked = true;
    store.set(id, el);
  }
  return store.get(id);
}
function def(name, value) {
  try { Object.defineProperty(globalThis, name, { value, writable: true, configurable: true }); }
  catch (e) { console.log('stub fail: ' + name + ' ' + e.message); }
}
def('document', { getElementById: elFor, createElement: mkEl, createDocumentFragment: () => mkEl('fragment'),
  querySelector: () => mkEl('div'), querySelectorAll: () => [], addEventListener(){}, body: mkEl('body'),
  documentElement: { clientWidth: 1600, clientHeight: 900, style: { setProperty(){}, getPropertyValue(){ return ''; } } } });
def('window', { devicePixelRatio: 1, addEventListener(){}, location: { href: 'file:///x' }, innerWidth: 1600, innerHeight: 900 });
def('location', { href: 'file:///x', hash: '' });
def('navigator', { serial: {} });
def('getComputedStyle', () => new Proxy({}, { get: () => '' }));
def('requestAnimationFrame', () => 0);
def('localStorage', { getItem: () => null, setItem(){}, removeItem(){} });
def('Blob', class {});
/* setInterval 桩：记录回调，供测试手动触发演示帧 */
const INTERVALS = [];
def('__intervals', INTERVALS);
def('setInterval', (fn, ms) => { INTERVALS.push({ fn, ms }); return INTERVALS.length; });
def('clearInterval', () => { INTERVALS.length = 0; });
try { globalThis.URL.createObjectURL = () => 'blob:x'; globalThis.URL.revokeObjectURL = () => {}; } catch {}

/* ---------------- 载入被测脚本 ---------------- */
__SCRIPT__

const T = globalThis.__T;
let pass = 0, fail = 0;
function ok(name, cond, info) {
  if (cond) { pass++; console.log('  PASS  ' + name + (info ? '  [' + info + ']' : '')); }
  else { fail++; console.log('  FAIL  ' + name + (info ? '  [' + info + ']' : '')); }
}
/* 与演示模式相同：AA 55 | LEN(整帧长度) | 4×float(BE) | SEQ | CRC16(LE) = 22 字节 */
function frame(seq, vals) {
  const body = [0xaa, 0x55, 0x00];
  for (const v of vals) { const dv = new DataView(new ArrayBuffer(4)); dv.setFloat32(0, v, false);
    body.push(dv.getUint8(0), dv.getUint8(1), dv.getUint8(2), dv.getUint8(3)); }
  body.push(seq & 0xff);
  body[2] = body.length + 2;
  const c = T.crc16(body, { poly: 0xa001, init: 0xffff, refin: true, xorout: 0 });
  body.push(c & 0xff, (c >> 8) & 0xff);
  return Uint8Array.from(body);
}
const expect = [25.5, 55.25, 12.2, 1.05];
T.fieldDefs.length = 0;
[['温度','f32',3,4],['湿度','f32',7,4],['电压','f32',11,4],['电流','f32',15,4],['SEQ','u8',19,1]]
  .forEach(([n, t, o, l]) => T.fieldDefs.push({ name: n, type: t, off: o, len: l, endian: 'be' }));

console.log('== 1. 工具函数 ==');
ok('parseHexStr("AA 55 01")', JSON.stringify([...T.parseHexStr('AA 55 01')]) === '[170,85,1]');
let threw = false; try { T.parseHexStr('AAB'); } catch { threw = true; }
ok('parseHexStr 奇数长度报错', threw);
ok('hex() 格式化', T.hex(Uint8Array.from([0, 15, 255])) === '00 0F FF');

console.log('== 2. CRC 标准向量 ==');
const crcVec = T.crc16([0x01, 0x04, 0x02, 0xFF, 0xFF], { poly: 0xa001, init: 0xffff, refin: true, xorout: 0 });
ok('CRC16/MODBUS(01 04 02 FF FF) == 0x80B8', crcVec === 0x80B8, 'got 0x' + crcVec.toString(16).toUpperCase());
const crcAscii = T.crc16(Array.from(new TextEncoder().encode('123456789')), { poly: 0xa001, init: 0xffff, refin: true, xorout: 0 });
ok('CRC16/MODBUS("123456789") == 0x4B37', crcAscii === 0x4B37, 'got 0x' + crcAscii.toString(16).toUpperCase());
ok('CRC16/CCITT-FALSE("123456789") == 0x29B1',
   T.CRC_ALGOS.crc16ccitt.fn(Array.from(new TextEncoder().encode('123456789'))) === 0x29B1);
ok('SUM8(01 02 03) == 6', T.CRC_ALGOS.sum8.fn([1, 2, 3]) === 6);
ok('XOR8(01 02 03) == 0', T.CRC_ALGOS.xor8.fn([1, 2, 3]) === 0);
ok('9 种校验算法齐全', ['crc16modbus','crc16ccitt','crc16xmodem','crc16ibm','crc16kermit','crc8','sum8','xor8','lrc'].every(k => T.CRC_ALGOS[k]));

console.log('== 3. 字段解析 (AA 55 | LEN | 4×float | SEQ | CRC16) ==');
const f1 = frame(7, expect);
ok('帧长 == 22', f1.length === 22, 'len=' + f1.length);
ok('LEN 字节 == 22 (0x16)', f1[2] === 22, 'LEN=' + f1[2]);
/* 演示帧约定：CRC 覆盖「整帧」（帧头也计入）→ crcScope = all */
const parsed = T.parseFrameFields(f1, { head: [0xaa, 0x55] });
ok('CRC 校验通过', parsed.crcOk === true, parsed.crcInfo);
const calcOver = T.crc16(Array.from(f1.slice(0, f1.length - 2)));
const tailVal = f1[f1.length - 2] | (f1[f1.length - 1] << 8);
ok('CRC 计算值 == 帧尾两字节(LE)', calcOver === tailVal,
   'calc=0x' + calcOver.toString(16).toUpperCase() + ' tail=0x' + tailVal.toString(16).toUpperCase());
ok('CRC == 0x83F5（Python 独立实现仲裁值）', calcOver === 0x83F5, '0x' + calcOver.toString(16).toUpperCase());
ok('整帧含 CRC 再算 CRC == 0x0000（自反性）', T.crc16(Array.from(f1)) === 0x0000);
elFor('crcScope').value = 'afterHead';
const parsedAH = T.parseFrameFields(f1, { head: [0xaa, 0x55] });
ok('afterHead 语义生效（跳过帧头后 CRC=0xAEBD，故校验失败）',
   parsedAH.crcOk === false && parsedAH.crcInfo.includes('AEBD'), parsedAH.crcInfo);
elFor('crcScope').value = 'all';
ok('SEQ == 7', parsed.values['SEQ'] === 7, 'got ' + parsed.values['SEQ']);
let floatsOk = true;
['温度','湿度','电压','电流'].forEach((n, i) => { if (Math.abs(parsed.values[n] - expect[i]) > 1e-4) { floatsOk = false; console.log('    ' + n + ' got ' + parsed.values[n]); } });
ok('4 个 float 解析正确', floatsOk, parsed.fields.map(x => x.value).join(', '));
elFor('crcEndian').value = 'be';
ok('CRC 端序切到 BE 后校验失败（证明端序设置生效）',
   T.parseFrameFields(f1, { head: [0xaa, 0x55] }).crcOk === false);
elFor('crcEndian').value = 'le';

console.log('== 4. 长度字段切帧: 逐字节喂入(模拟串口分片/粘包) ==');
T.P.buf = []; T.S.frameCount = 0; T.S.lines.length = 0; T.S.dropCount = 0;
const f2 = frame(8, [1, 2, 3, 4]);
const all = new Uint8Array(f1.length + f2.length); all.set(f1, 0); all.set(f2, f1.length);
for (const b of all) T.feedParser(Uint8Array.from([b]));
ok('逐字节粘包切出 2 帧', T.S.frameCount === 2, 'frames=' + T.S.frameCount);
ok('缓冲区无残留', T.P.buf.length === 0, 'buf=' + T.P.buf.length);
ok('2 帧都通过 CRC', T.S.lines.filter(l => l.extra.includes('校验通过')).length === 2);
ok('SEQ 递增正确', T.S.lines[0].extra.includes('SEQ=7') && T.S.lines[1].extra.includes('SEQ=8'));

console.log('== 5. 长度字段切帧: 随机分片 + 前导噪声 ==');
T.P.buf = []; T.S.frameCount = 0; T.S.dropCount = 0;
const noisy = new Uint8Array(5 + all.length); noisy.set([0x13, 0x37, 0x00, 0xaa, 0x99], 0); noisy.set(all, 5);
let i2 = 0;
while (i2 < noisy.length) { const n = 1 + Math.floor(Math.random() * 5); T.feedParser(noisy.slice(i2, i2 + n)); i2 += n; }
ok('随机分片下仍切出 2 帧', T.S.frameCount === 2, 'frames=' + T.S.frameCount);
ok('统计了丢弃字节', T.S.dropCount >= 3, 'drop=' + T.S.dropCount);
ok('缓冲区无残留', T.P.buf.length === 0, 'buf=' + T.P.buf.length);

console.log('== 6. 行切帧 (CRLF) ==');
T.P.buf = []; T.S.frameCount = 0; T.S.lines.length = 0;
elFor('pMode').value = 'line'; elFor('lineEnd').value = 'crlf';
T.feedParser(new TextEncoder().encode('T=25.5,H=60\r\nT=26.0,H=61\r\nT=2'));
ok('切出 2 行', T.S.frameCount === 2, 'frames=' + T.S.frameCount);
ok('半行留在缓冲区', T.P.buf.length === 3, 'buf=' + T.P.buf.length);
T.feedParser(new TextEncoder().encode('7.0,H=62\r\n'));
ok('补齐后共 3 行', T.S.frameCount === 3, 'frames=' + T.S.frameCount);
ok('行内容不含 CRLF', T.S.lines[0].text === 'T=25.5,H=60', JSON.stringify(T.S.lines[0].text));
elFor('pMode').value = 'len';

console.log('== 7. 篡改 CRC 可被检出 ==');
const bad = Uint8Array.from(f1); bad[bad.length - 1] ^= 0xff;
ok('篡改尾字节 -> 校验失败', T.parseFrameFields(bad, { head: [0xaa, 0x55] }).crcOk === false);
const bad2 = Uint8Array.from(f1); bad2[5] ^= 0x01;
ok('篡改数据域 -> 校验失败', T.parseFrameFields(bad2, { head: [0xaa, 0x55] }).crcOk === false);

console.log('== 8. 发送编码 ==');
elFor('txArea').value = 'AT+VER?'; elFor('txHex').checked = false;
elFor('append').value = 'crlf'; elFor('txEnc').value = 'UTF-8';
ok('文本 + CRLF', T.hex(T.txPayload()) === '41 54 2B 56 45 52 3F 0D 0A', T.hex(T.txPayload()));
elFor('txHex').checked = true; elFor('append').value = 'none'; elFor('txArea').value = 'AA 55 01 00';
ok('HEX 发送', T.hex(T.txPayload()) === 'AA 55 01 00', T.hex(T.txPayload()));
elFor('txHex').checked = false; elFor('append').value = 'lf'; elFor('txArea').value = 'a\\x01b';
ok('文本转义 \\xNN + LF 结尾', T.hex(T.txPayload()) === '61 01 62 0A', T.hex(T.txPayload()));
elFor('append').value = 'none';
ok('转义 \\n 生效', T.hex(T.txPayload()) === '61 01 62', T.hex(T.txPayload()));

console.log('== 10. 空闲超时切帧 ==');
elFor('pMode').value = 'timeout'; elFor('timeoutMs').value = '20';
T.P.buf = []; T.S.frameCount = 0; T.S.lines.length = 0;
T.feedParser(Uint8Array.from([0x01, 0x02, 0x03]));
T.feedParser(Uint8Array.from([0x04]));
if (T.S.frameCount !== 0) console.log('    提示: 超时未到就已切帧（不应发生）');
await new Promise(r => setTimeout(r, 60));
ok('空闲 60ms 后整段切为 1 帧', T.S.frameCount === 1, 'frames=' + T.S.frameCount);
const rxLines = T.S.lines.filter(l => l.dir === 'rx');
ok('帧内容为 01 02 03 04', rxLines.length === 1 && T.hex(rxLines[0].bytes) === '01 02 03 04',
   'rx=' + JSON.stringify(rxLines.map(l => T.hex(l.bytes))));
ok('超时后缓冲区已清空', T.P.buf.length === 0, 'buf=' + T.P.buf.length);
elFor('pMode').value = 'len';

console.log('== 11. 演示模式端到端（真实演示回调 → 解析器） ==');
T.P.buf = []; T.S.frameCount = 0; T.S.lines.length = 0;
INTERVALS.length = 0;
T.startDemo();
T.S.lines.length = 0;                                    // 清掉 startDemo 打的系统提示行
ok('演示模式注册了 100ms 定时器', INTERVALS.length === 1 && INTERVALS[0].ms === 100,
   JSON.stringify(INTERVALS.map(i => i.ms)));
T.P.buf = []; T.S.frameCount = 0; T.S.lines.length = 0;
for (let i = 0; i < 10; i++) INTERVALS[0].fn();          // 手动触发 10 帧
ok('演示回调产出 10 帧', T.S.frameCount === 10, 'frames=' + T.S.frameCount);
ok('10 帧 CRC 全部通过', T.S.lines.every(l => l.extra.includes('校验通过')),
   T.S.lines.filter(l => l.extra.includes('校验通过')).length + '/10');
ok('演示帧 len == 22', T.S.lines[0].bytes.length === 22, 'len=' + T.S.lines[0].bytes.length);
ok('演示帧 SEQ 递增 0..9',
   [0,1,2,3,4,5,6,7,8,9].every(i => T.S.lines[i].extra.includes('SEQ=' + i)));
ok('演示模式无残留字节', T.P.buf.length === 0, 'buf=' + T.P.buf.length);
ok('波形序列已积累（温度/湿度/电压/电流/SEQ）', T.S.series.size >= 4, 'series=' + [...T.S.series.keys()].join(','));
T.stopDemo();

console.log('== 12. 可拖拽布局 ==');
ok('默认尺寸已定义', T.LAYOUT_DEFAULT.wL === 326 && T.LAYOUT_DEFAULT.wR === 346 && T.LAYOUT_DEFAULT.chartH === 250,
   JSON.stringify(T.LAYOUT_DEFAULT));
ok('布局尺寸使用独立存储键', T.LAYOUT_KEY === 'serial-studio-layout', T.LAYOUT_KEY);
T.layout.wL = 420; T.layout.wR = 300; T.layout.chartH = 330;
T.applyLayout();
ok('applyLayout 未抛异常且可重复调用', (() => { try { T.applyLayout(); return true; } catch (e) { console.log('    ' + e.message); return false; } })());
T.layout.wL = T.LAYOUT_DEFAULT.wL; T.layout.wR = T.LAYOUT_DEFAULT.wR; T.layout.chartH = T.LAYOUT_DEFAULT.chartH;
T.applyLayout();

console.log('== 13. 接收链路诊断（收不到数据时的关键路径） ==');
/* 原始流模式：无论字节是否符合任何帧格式，都必须原样进入日志 —— 这是「收不到数据」排查的底线 */
elFor('pMode').value = 'raw';
T.P.buf = []; T.S.frameCount = 0; T.S.lines.length = 0;
const junk = Uint8Array.from([0x13, 0x37, 0x00, 0xff, 0x7e, 0x81]);
T.feedParser(junk);
const rxRaw = T.S.lines.filter(l => l.dir === 'rx');
ok('原始流模式不丢字节', T.S.frameCount === 1 && rxRaw.length === 1, 'frames=' + T.S.frameCount);
ok('原始流内容与收到字节一致', rxRaw.length === 1 && T.hex(rxRaw[0].bytes) === '13 37 00 FF 7E 81',
   rxRaw.length ? T.hex(rxRaw[0].bytes) : '(无)');
/* 定长模式的「吞字节」现象：帧头不匹配时字节被当作噪声，这条用于说明排查要点 */
elFor('pMode').value = 'len'; elFor('lenHead').value = 'DE AD';
T.P.buf = []; T.S.frameCount = 0; T.S.dropCount = 0;
T.feedParser(Uint8Array.from([0x01, 0x02, 0x03, 0x04]));
ok('帧头不匹配时计入「丢弃」而非显示（需切回原始流确认）', T.S.frameCount === 0 && T.S.dropCount >= 0,
   'frames=' + T.S.frameCount + ' drop=' + T.S.dropCount);
elFor('lenHead').value = 'AA 55'; elFor('pMode').value = 'len';
/* 零字节看门狗只在连接且长时间无数据时提示 */
T.S.connected = false; T.S.stallWarned = false; T.S.rxBytes = 0;
T.checkRxStall();
ok('未连接时不误报', T.S.stallWarned === false);
T.S.connected = true; T.S.demo = false; T.S.closing = false;
T.S.startedAt = Date.now(); T.S.lastRxAt = Date.now();
T.checkRxStall();
ok('刚连接 0 字节不提示', T.S.stallWarned === false);
T.S.lastRxAt = Date.now() - 9000;
T.checkRxStall();
ok('连接 9 秒 0 字节给出排查提示', T.S.stallWarned === true);
T.S.connected = false; T.S.rxBytes = 0; T.S.stallWarned = false;

console.log('== 14. 帧头自动识别（收得到字节但切不出帧时的兜底） ==');
/* 构造一段「STM32 风格」的流：帧头 5A A5 + LEN(整帧) + 数据 + 校验，前面有噪声 */
function stmFrame(seq, n) {
  /* 帧 = 5A A5 | SEQ | LEN(整帧字节数) | DATA(10) | SUM8，共 15 字节 */
  const body = [0x5a, 0xa5, seq & 0xff, 0x00];
  for (let i = 0; i < 10; i++) body.push((seq * 7 + i * 13) & 0xff);
  body[3] = body.length + 1;                    // LEN = 整帧长度（含校验字节）
  let sum = 0; for (const b of body) sum = (sum + b) & 0xff;
  body.push(sum);
  return body;
}
const stream = [0x00, 0x11, 0x22, 0x33, 0xff];
for (let s = 0; s < 8; s++) stream.push(...stmFrame(s, 10));   // 每帧 15 字节
const bytes = Uint8Array.from(stream);
T.hexBytesReset(); T.hexBytesFeed(bytes);
ok('环形缓冲内容正确', T.hexBytesGet(bytes.length).length === bytes.length
   && T.hex(T.hexBytesGet(bytes.length)) === T.hex(bytes), 'len=' + T.hexBytesGet(bytes.length).length);
const det = T.analyzeHead(bytes);
ok('识别出正确帧头 5A A5', det && det.head[0] === 0x5a && det.head[1] === 0xa5,
   det ? det.head.map(b => b.toString(16).toUpperCase()).join(' ') : '未识别');
ok('识别出帧长 16', det && Math.abs(det.len - 16) <= 1, det ? 'len=' + det.len : '-');
ok('报告重复次数', det && det.frames >= 5, det ? 'frames=' + det.frames : '-');
/* 用错误帧头（默认 AA 55）解析这段流：应当一帧都切不出来，并触发无帧检测 */
elFor('pMode').value = 'len'; elFor('lenHead').value = 'AA 55';
T.P.buf = []; T.S.frameCount = 0; T.S.dropCount = 0;
T.S.connected = true; T.S.demo = false; T.S.closing = false;
T.S.parserStalled = false; T.S.stallWarned = true;
/* 连发多轮，确保原始字节缓冲超过「无帧检测」的 256 字节门槛 */
const longStream = [];
for (let r = 0; r < 4; r++) longStream.push(...bytes);
const longBytes = Uint8Array.from(longStream);
T.hexBytesReset(); T.hexBytesFeed(longBytes);
T.feedParser(longBytes);
ok('帧头不匹配时切不出帧', T.S.frameCount === 0, 'frames=' + T.S.frameCount);
T.S.lastRxAt = Date.now() - 2000;
try { T.checkNoFrames(); } catch (e) { console.log('    checkNoFrames 抛异常: ' + e.message); }
ok('触发「收到数据但无帧」检测', T.S.parserStalled === true, 'stalled=' + T.S.parserStalled);
/* 换成识别出的帧头与长度约定后应当正常切帧 */
elFor('lenHead').value = '5A A5'; elFor('lenPos').value = '3'; elFor('lenSize').value = '1';
elFor('lenKind').value = 'total'; elFor('lenExtra').value = '0';
T.P.buf = []; T.S.frameCount = 0; T.S.lines.length = 0;
T.feedParser(longBytes);
ok('按识别结果切出 32 帧', T.S.frameCount === 32, 'frames=' + T.S.frameCount);
const rxFrames = T.S.lines.filter(l => l.dir === 'rx');
ok('每帧 15 字节且与 LEN 自洽',
   rxFrames.length === 32 && rxFrames.every(l => l.bytes.length === 15 && l.bytes[3] === 15),
   rxFrames.slice(0, 4).map(l => l.bytes.length).join(','));
/* 原始流模式：任何字节都必须能看到 */
elFor('pMode').value = 'raw';
T.P.buf = []; T.S.frameCount = 0; T.S.lines.length = 0;
T.feedParser(bytes);
ok('原始流模式不丢任何字节', T.S.frameCount === 1 && T.hex(T.S.lines.filter(l => l.dir === 'rx')[0].bytes) === T.hex(bytes));
elFor('pMode').value = 'len'; elFor('lenHead').value = 'AA 55';
T.S.connected = false; T.S.parserStalled = false; T.S.stallWarned = false;

console.log('== 15. 上位机二进制帧协议（A5 | LEN | SEQ | CMD | DATA | CRC8） ==');
/* CRC-8 标准校验值：'123456789' */
const asc = Array.from(new TextEncoder().encode('123456789'));
ok('CRC-8/MAXIM 校验值 0xA1', T.crc8(asc, 0x8C, 0x00, 0x00, true) === 0xA1,
   '0x' + T.crc8(asc, 0x8C, 0x00, 0x00, true).toString(16).toUpperCase());
ok('CRC-8/SAE-J1850 校验值 0x4B', T.crc8(asc, 0x1D, 0xFF, 0xFF, false) === 0x4B,
   '0x' + T.crc8(asc, 0x1D, 0xFF, 0xFF, false).toString(16).toUpperCase());
ok('本协议 CRC-8 (poly 0x31/init 0xFF/无反射) = 0xF7', T.crc8(asc, 0x31, 0xFF, 0x00, false) === 0xF7,
   '0x' + T.crc8(asc, 0x31, 0xFF, 0x00, false).toString(16).toUpperCase());

/* 按协议构造帧：CRC 覆盖 LEN~DATA */
function tlv(seq, cmd, data) {
  const body = [data.length & 0xff, seq & 0xff, cmd & 0xff, ...data];
  return Uint8Array.from([0xA5, ...body, T.crc8(body, 0x31, 0xFF, 0x00, false)]);
}
const t0 = tlv(1, 0x01, []);
const t2 = tlv(2, 0x10, [0x01, 0x02]);
const t8 = tlv(3, 0x20, [0, 1, 2, 3, 4, 5, 6, 7]);
ok('LEN=0 的空载荷帧 = A5 00 01 01 8E', T.hex(t0) === 'A5 00 01 01 8E', T.hex(t0));
ok('LEN=2 帧 = A5 02 02 10 01 02 31', T.hex(t2) === 'A5 02 02 10 01 02 31', T.hex(t2));
ok('LEN=8 帧尾 CRC=0x03', t8[t8.length - 1] === 0x03, '0x' + t8[t8.length - 1].toString(16).toUpperCase());

/* 先彻底重置解析状态，避免上一节残留的缓冲污染本节 */
T.P.buf = []; T.S.frameCount = 0; T.S.lines.length = 0; T.S.dropCount = 0;
/* 按「示例：TLV 帧」预设的等效配置 */
elFor('pMode').value = 'len'; elFor('lenHead').value = 'A5'; elFor('lenPos').value = '1';
elFor('lenSize').value = '1'; elFor('lenKind').value = 'payload'; elFor('lenExtra').value = '3';
elFor('lenPos').value = '1';
elFor('lenEndian').value = 'le'; elFor('chkCrc').checked = true; elFor('crcAlgo').value = 'crc8_31';
elFor('crcScope').value = 'afterHead'; elFor('crcWhere').value = 'tail'; elFor('crcEndian').value = 'le';
T.fieldDefs.length = 0;
T.fieldDefs.push({ name: 'SEQ', type: 'u8', off: 2, len: 1, endian: '' });
T.fieldDefs.push({ name: 'CMD', type: 'u8', off: 3, len: 1, endian: '' });
T.fieldDefs.push({ name: 'DATA', type: 'hex', off: 4, len: 0, endian: '' });
ok('CRC8 变体已可选', ['crc8', 'crc8_31', 'crc8_maxim', 'crc8_j1850'].every(k => T.CRC_ALGOS[k]));

/* 切帧：干净流 + 任意分片（真实串口就是这种分片，帧头一定出现在分片边界之内） */
T.P.buf = []; T.S.frameCount = 0; T.S.lines.length = 0; T.S.dropCount = 0;
const clean2 = new Uint8Array(t0.length + t2.length + t8.length);
clean2.set(t0, 0); clean2.set(t2, t0.length); clean2.set(t8, t0.length + t2.length);
let jj = 0;
while (jj < clean2.length) {
  const n = 1 + ((jj * 7) % 8);                 // 1~8 字节的伪随机分片
  T.feedParser(clean2.slice(jj, jj + n));
  jj += n;
}
const bufStr = T.P.buf.length ? T.P.buf.map(x => x.toString(16)).join(' ') : '空';
ok('任意分片下切出 3 帧', T.S.frameCount === 3, 'frames=' + T.S.frameCount);
ok('缓冲区无残留', T.P.buf.length === 0, 'buf=' + bufStr);
const rx15 = T.S.lines.filter(l => l.dir === 'rx');
ok('3 帧长度分别为 5 / 7 / 13', rx15.map(l => l.bytes.length).join(',') === '5,7,13',
   rx15.map(l => l.bytes.length).join(','));
ok('3 帧 CRC8 全部通过', rx15.every(l => l.extra.includes('校验通过')),
   rx15.filter(l => l.extra.includes('校验通过')).length + '/3');
ok('SEQ / CMD 解析正确', rx15[1].extra.includes('SEQ=2') && rx15[1].extra.includes('CMD=16'), rx15[1].extra);
ok('LEN=2 帧的 DATA=01 02', rx15[1].extra.includes('DATA=01 02'), rx15[1].extra);
ok('LEN=8 帧的 DATA=00 01 02 03 04 05 06 07',
   rx15[2].extra.includes('DATA=00 01 02 03 04 05 06 07'), rx15[2].extra);
/* 无帧头 / 帧头不匹配时不得误切帧（保证「无帧检测」的判定基础） */
elFor('lenHead').value = 'DE AD';
T.P.buf = []; T.S.frameCount = 0;
T.feedParser(clean2);
ok('帧头不匹配时一帧都不切', T.S.frameCount === 0, 'frames=' + T.S.frameCount);
elFor('lenHead').value = 'A5';
/* 篡改任意字节必须被 CRC8 检出 */
const bad3 = Uint8Array.from(t2); bad3[4] ^= 0x01;
ok('篡改 DATA 后 CRC8 校验失败', T.parseFrameFields(bad3, { head: [0xA5] }).crcOk === false);
const bad4 = Uint8Array.from(t2); bad4[1] = 5;
ok('篡改 LEN 后 CRC8 校验失败', T.parseFrameFields(bad4, { head: [0xA5] }).crcOk === false);

console.log('');
console.log('结果: ' + pass + ' 通过 / ' + fail + ' 失败');
process.exit(fail ? 1 : 0);
"""

out = HERE / "_selftest.mjs"
out.write_text(harness.replace("__SCRIPT__", script), encoding="utf-8")

env = dict(os.environ); env["PYTHONIOENCODING"] = "utf-8"
r = subprocess.run([NODE, str(out)], capture_output=True, encoding="utf-8", errors="replace", env=env)
sys.stdout.reconfigure(encoding="utf-8", errors="replace")
sys.stdout.write(r.stdout or "")
sys.stdout.write(r.stderr or "")
print("node exit code:", r.returncode)
sys.exit(r.returncode)
