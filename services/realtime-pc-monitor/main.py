"""
EPD CPU/GPU Monitor - gửi thông số CPU/GPU lên màn e-paper 122x250 qua BLE
Tương thích với firmware EPD-nRF5 (cùng giao thức với main.js của web app gốc).

Yêu cầu cài đặt (Windows 11, PowerShell):
    pip install bleak pillow psutil

Nếu có GPU NVIDIA và muốn đọc % GPU (gói pynvml đã deprecated, dùng gói thay thế):
    pip install nvidia-ml-py

CÁCH LẤY EPD_PINS_HEX / EPD_DRIVER_HEX:
    1. Mở web app gốc, kết nối Bluetooth với màn hình như bình thường (1 lần).
    2. Sau khi kết nối, web app sẽ tự đọc cấu hình hiện tại của thiết bị và điền
       vào 2 ô input "epdpins" và "epddriver" trên giao diện (mở DevTools ->
       Elements, hoặc Console gõ document.getElementById('epdpins').value và
       document.getElementById('epddriver').value để xem).
    3. Copy 2 chuỗi hex đó vào EPD_PINS_HEX và EPD_DRIVER_HEX bên dưới.
    Nếu để trống, script sẽ bỏ qua bước SET_PINS/đổi driver và chỉ gửi INIT,
    dùng cấu hình driver hiện đang lưu sẵn trong firmware (thường là đủ nếu bạn
    đã từng cấu hình đúng qua web app trước đó).

ĐỊNH DẠNG BITMAP (đã xác nhận qua ảnh chụp UI web app: driver/màn hình của bạn
đang ở chế độ 颜色模式 = "三色(黑白红)" = threeColor, KHÔNG phải blackWhiteColor!
Port 1-1 từ processImageData() mode 'threeColor' trong dithering.js gốc):
    - Gồm 2 mặt phẳng (plane) byte riêng biệt, mỗi plane 1bpp MSB-first,
      byteWidth = ceil(width/8):
        + blackWhiteData: bit=1 trắng, bit=0 đen (threshold grayscale 140)
        + redWhiteData:   bit=0 nếu pixel đỏ rõ rệt (r>160 và r>g, r>b), bit=1 nếu không
    - Gửi 2 mặt phẳng này bằng 2 lệnh WRITE_IMG riêng (step='bw' rồi step='red'),
      sau đó mới REFRESH 1 lần - giống hệt sendimg() trong main.js gốc:
        await writeImage(blackWhiteData, 'bw');
        await writeImage(redWhiteData, 'red');
        await write(REFRESH);
    Script này chỉ vẽ đen/trắng (không dùng đỏ) nên redWhiteData sẽ tự động toàn
    bit=1 (không đỏ), nhưng VẪN PHẢI gửi đủ cả 2 plane vì firmware mong đợi đủ
    dữ liệu - thiếu plane đỏ chính là nguyên nhân gây nhiễu "sạn đen đỏ" trước đó.
"""

import asyncio
import os
import sys
import time
from datetime import datetime

import psutil
from PIL import Image, ImageDraw, ImageFont
from bleak import BleakClient, BleakScanner

# ============== CẤU HÌNH ==============

EPD_WIDTH = 122
EPD_HEIGHT = 250

# Lấy từ web app (xem hướng dẫn ở đầu file). Để '' nếu không cần set lại.
EPD_PINS_HEX = "1413060504030207"    # ví dụ: "0a0b0c0d0e0f10"
EPD_DRIVER_HEX = "02"  # ví dụ: "01" 4.2-inch (tri-color, SSD1619)

UPDATE_INTERVAL_SEC = 300  # chu kỳ refresh; e-paper full refresh chậm & hao mòn nên đừng để quá ngắn
INTERLEAVED_COUNT = 50    # đã test thực tế trên web app: MTU=244, confirm interval=50 -> chạy hoàn hảo

# Đã xác nhận bằng thực nghiệm: web app gửi ảnh mượt với MTU=244 (chunk_size = 244-2 = 242)
# và interleaved=50. Dùng đúng thông số này thay vì đoán mò.
AUTO_DETECT_MTU = False
SAFE_CHUNK_SIZE = 242

# Bật chế độ này để KHÔNG kết nối BLE, chỉ render ảnh CPU/GPU và lưu ra file
# debug_output.png để bạn tự kiểm tra / upload thủ công lên web app gốc thử.
DEBUG_SAVE_ONLY = False
DEBUG_OUTPUT_PATH = "debug_output.png"

SERVICE_UUID = "62750001-d828-918d-fb46-b6c11c675aec"
CHAR_UUID = "62750002-d828-918d-fb46-b6c11c675aec"
VERSION_CHAR_UUID = "62750003-d828-918d-fb46-b6c11c675aec"

DEVICE_NAME_FILTER = 'NRF_EPD_8687'  # NRF_EPD_8687 ví dụ "EPD-xxxx" để lọc nhanh khi scan, để None để chọn thủ công
DEVICE_ADDRESS = None # 58593A56-6ED3-2247-3E7A-51C2B785C291 "DC:98:5A:4C:86:87"
CACHE_TXT = "device_address.txt"
MODE = 1 # 1: image, 2: calendar, 3: clock
SCAN_MODE = 0 # 0: fixed device, 1: scan all devices

# Màn hình vật lý của bạn đặt NGANG (landscape) chứ không dọc. Buffer gửi
# xuống firmware vẫn phải đúng kích thước EPD_WIDTH x EPD_HEIGHT (122x250,
# khớp canvas/driver đã cấu hình), nên ta vẽ nội dung trên 1 canvas "ảo" nằm
# ngang rồi xoay lại cho khớp buffer thật trước khi gửi.
# Thử 90 trước; nếu lên màn bị ngược/lộn thì đổi thành -90 (hoặc 270).
ROTATE_FOR_LANDSCAPE = 90  # 0 = không xoay (giữ dọc), 90 / -90 / 180 / 270

# Vẽ ở độ phân giải cao hơn rồi thu nhỏ lại (supersampling) để chữ đỡ vỡ/răng cưa
# khi xuống còn màn 1-bit độ phân giải thấp.
SUPERSAMPLE = 1

# Sau khi thu nhỏ xLANCZOS), nét chữ mảnh có thể chỉ còn là xám nhạt thay vì đen
# đặc -> dễ bị threshold coi là "trắng" và biến mất hoàn toàn. Threshold dưới
# đây dùng RIÊNG cho bước nhị phân hóa chữ (không phải threshold encode protocol),
# cố tình nghiêng về phía ĐEN để giữ lại nét chữ mảnh: pixel có độ sáng dưới
# giá trị này mới được tính là đen, ngược lại trắng. Giảm xuống nếu chữ vẫn bị
# đứt nét, tăng lên nếu chữ bị quá đậm/dính chữ.
TEXT_THRESHOLD = 220

# Định dạng đã xác nhận khớp 100% với code gốc, không cần chỉnh các cờ debug nữa.

# ============== EpdCmd (giống main.js) ==============

class EpdCmd:
    SET_PINS = 0x00
    INIT = 0x01
    CLEAR = 0x02
    SEND_CMD = 0x03
    SEND_DATA = 0x04
    REFRESH = 0x05
    SLEEP = 0x06
    SET_TIME = 0x20
    WRITE_IMG = 0x30
    CALENDAR_MODE = 1
    CLOCK_MODE = 2


def hex2bytes(hexstr: str) -> bytes:
    return bytes.fromhex(hexstr) if hexstr else b""


# ============== Lấy số liệu CPU/GPU ==============

def get_cpu_percent() -> float:
    return psutil.cpu_percent(interval=0.5)


def get_gpu_percent():
    """
    Lấy % sử dụng GPU bằng cách truy vấn lớp Win32_PerfFormattedData_GPUPerformanceCounters.
    """
    if os.name != "nt":
        return None

    try:
        import subprocess
        startupinfo = subprocess.STARTUPINFO()
        startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW

        # Lấy dữ liệu thô từ cấu trúc hiệu năng của WMI Windows
        cmd = "wmic path Win32_PerfFormattedData_GPUPerformanceCounters_GPUEngine get UtilizationPercentage"
        res = subprocess.check_output(cmd, text=True, startupinfo=startupinfo)
        
        utils = []
        for line in res.splitlines():
            line = line.strip()
            if line.isdigit():
                val = int(line)
                if 0 < val <= 100:  # Lọc bỏ giá trị ảo
                    utils.append(val)
                    
        if utils:
            return max(utils)  # Trả về % của engine đang hoạt động cao nhất
            
        return 0
    except Exception as e:
        print(f"Không đọc được AMD GPU qua WMI ({e})")
        return None


# ============== Render ảnh CPU/GPU thành bitmap 1bpp ==============

def load_font(size, bold=False):
        candidates = []

        if os.name == "nt":
            windir = os.environ.get("WINDIR", r"C:\Windows")
            fonts = os.path.join(windir, "Fonts")

            if bold:
                candidates += [
                    os.path.join(fonts, "arialbd.ttf"),
                    os.path.join(fonts, "segoeuib.ttf"),
                    os.path.join(fonts, "calibrib.ttf"),
                ]
            else:
                candidates += [
                    os.path.join(fonts, "arial.ttf"),
                    os.path.join(fonts, "segoeui.ttf"),
                    os.path.join(fonts, "calibri.ttf"),
                ]

        else:
            candidates += [
                "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                "/Library/Fonts/Arial.ttf",
                "/System/Library/Fonts/Supplemental/Arial.ttf",
            ]

        for f in candidates:
            try:
                return ImageFont.truetype(f, size)
            except Exception:
                pass

        return ImageFont.load_default()
    
    
def render_stats_image() -> Image.Image:
    # RGB vì driver thực tế của bạn là 3 màu (đen/trắng/đỏ) - xem threeColor encode bên dưới.
    # Ở đây chỉ vẽ đen/trắng (không dùng màu đỏ), nhưng vẫn phải qua đúng pipeline
    # threeColor để khớp với firmware/driver đang cấu hình.
    cpu_pct = get_cpu_percent()
    gpu_pct = get_gpu_percent()
    print(f"[{datetime.now().strftime('%H:%M:%S')}] CPU={cpu_pct:.0f}% GPU={gpu_pct}")
    # Kích thước canvas "ảo" lúc vẽ: nếu có xoay 90/270 thì canvas vẽ là ngang
    # (hoán đổi width/height so với buffer thật), nếu không xoay thì giữ nguyên.
    if ROTATE_FOR_LANDSCAPE in (90, -90, 270, -270):
        draw_w, draw_h = EPD_HEIGHT, EPD_WIDTH  # canvas ngang: 250 x 122
    else:
        draw_w, draw_h = EPD_WIDTH, EPD_HEIGHT

    big_w, big_h = draw_w * SUPERSAMPLE, draw_h * SUPERSAMPLE
    img = Image.new("RGB", (big_w, big_h), (255, 255, 255))
    draw = ImageDraw.Draw(img)

    s = SUPERSAMPLE
    font_huge = load_font(50 * s, bold=True)
    font_label = load_font(24 * s, bold=True)
    font_small = load_font(20 * s)
    BLACK = (0, 0, 0)

    def draw_centered(text, font, cx, cy):
        bbox = draw.textbbox((0, 0), text, font=font)
        tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
        x = cx - tw / 2 - bbox[0]
        y = cy - th / 2 - bbox[1]
        draw.text((x, y), text, font=font, fill=BLACK)

    # Bố cục NGANG: CPU bên trái, GPU bên phải, vạch dọc chia giữa, giờ ở dưới.
    half_w = big_w // 2
    label_y = int(big_h * 0.22)
    value_y = int(big_h * 0.50)

    draw_centered("CPU", font_label, half_w // 2, label_y)
    draw_centered(f"{cpu_pct:.0f}%", font_huge, half_w // 2, value_y)

    draw.line((half_w, int(big_h * 0.12), half_w, int(big_h * 0.82)), fill=BLACK, width=2 * s)

    draw_centered("GPU", font_label, half_w + half_w // 2, label_y)
    gpu_text = f"{gpu_pct:.0f}%" if gpu_pct is not None else "N/A"
    draw_centered(gpu_text, font_huge, half_w + half_w // 2, value_y)

    now_str = datetime.now().strftime("%H:%M:%S")
    draw_centered(now_str, font_small, big_w // 2, int(big_h * 0.93))

    # Thu nhỏ lại bằng LANCZOS để chữ mượt (anti-alias), rồi mới threshold sau.
    img = img.resize((draw_w, draw_h), Image.Resampling.LANCZOS)

    # Xoay về đúng hướng buffer thật mà firmware mong đợi.
    if ROTATE_FOR_LANDSCAPE not in (0,):
        img = img.rotate(ROTATE_FOR_LANDSCAPE, expand=True)

    assert img.size == (EPD_WIDTH, EPD_HEIGHT), f"Kích thước sau xoay {img.size} != ({EPD_WIDTH},{EPD_HEIGHT})"
    return img


def image_to_threecolor_planes(img: Image.Image):
    """Port chính xác 1-1 từ processImageData() mode 'threeColor' trong dithering.js gốc.
    Trả về (blackWhiteData, redWhiteData) - mỗi mảng là byteWidth*height byte, MSB-first.
    blackWhiteBit: 1 = trắng, 0 = đen (threshold 140 trên grayscale).
    redWhiteBit:   0 = đỏ (pixel đỏ rõ rệt), 1 = không đỏ (dùng giá trị bw plane)."""
    w, h = img.size
    byte_width = (w + 7) // 8
    bw_data = bytearray(byte_width * h)
    red_data = bytearray(byte_width * h)
    pixels = img.load()

    BW_THRESHOLD = 140
    RED_THRESHOLD = 160

    for yy in range(h):
        for xx in range(w):
            r, g, b = pixels[xx, yy][:3]
            grayscale = round(0.299 * r + 0.587 * g + 0.114 * b)
            byte_idx = yy * byte_width + (xx // 8)
            bit_idx = 7 - (xx % 8)

            bw_bit = 1 if grayscale >= BW_THRESHOLD else 0
            if bw_bit:
                bw_data[byte_idx] |= (1 << bit_idx)

            red_bit = 0 if (r > RED_THRESHOLD and r > g and r > b) else 1
            if red_bit:
                red_data[byte_idx] |= (1 << bit_idx)

    return bytes(bw_data), bytes(red_data)


# ============== Giao tiếp BLE (port từ main.js) ==============

class EpdDevice:
    def __init__(self, client: BleakClient):
        self.client = client
        self.chunk_size = SAFE_CHUNK_SIZE  # sẽ được cập nhật lại sau khi đọc MTU thật nếu AUTO_DETECT_MTU=True

    async def sync_time(self, mode: int):
        # Get current Unix timestamp as an integer (seconds since epoch)
        timestamp = int(time.time())
        
        # Calculate timezone offset in hours (Python's altzone/timezone is in seconds, sign is inverted compared to JS)
        # JS: positive offset means behind UTC (e.g., Boston is +240 minutes). 
        # Python: negative offset means behind UTC, so we calculate accordingly.
        is_dst = time.localtime().tm_isdst
        timezone_offset_hours = -int((time.altzone if is_dst else time.timezone) / 3600)

        # Construct the byte array
        data = bytes([
            (timestamp >> 24) & 0xFF,
            (timestamp >> 16) & 0xFF,
            (timestamp >> 8) & 0xFF,
            timestamp & 0xFF,
            timezone_offset_hours & 0xFF,  # Masking with 0xFF handles negative numbers gracefully in byte arrays
            mode
        ])
        
        # Assuming EpdCmd, write, and add_log are defined elsewhere in your Python script
        if await self.write(EpdCmd.SET_TIME, data):
            print("Time is synchronized!")
            print("Please do not perform any actions until the screen has finished refreshing.")
        
    async def write(self, cmd: int, data: bytes = b"", with_response: bool = True):
        payload = bytes([cmd]) + data
        await self.client.write_gatt_char(CHAR_UUID, payload, response=with_response)

    async def write_image(self, data: bytes, step: str = "bw"):
        chunk_size = self.chunk_size
        no_reply_count = INTERLEAVED_COUNT
        total = len(data)
        sent = 0
        idx = 0
        count = round(total / chunk_size)

        while sent < total:
            chunk = data[sent: sent + chunk_size]
            header = (0x0F if step == "bw" else 0x00) | (0x00 if sent == 0 else 0xF0)
            payload = bytes([header]) + chunk

            if no_reply_count > 0:
                await self.write(EpdCmd.WRITE_IMG, payload, with_response=False)
                no_reply_count -= 1
            else:
                await self.write(EpdCmd.WRITE_IMG, payload, with_response=True)
                no_reply_count = INTERLEAVED_COUNT

            sent += chunk_size
            idx += 1
            print(f"  gửi chunk {idx}/{count + 1}", end="\r")
        print()

    async def send_image(self, img: Image.Image):
        bw_data, red_data = image_to_threecolor_planes(img)
        await self.write(EpdCmd.INIT)
        await self.write_image(bw_data, "bw")
        await self.write_image(red_data, "red")
        await self.write(EpdCmd.REFRESH)
    
    async def sync_metrics(self):
        print(f"Bắt đầu vòng lặp cập nhật mỗi {UPDATE_INTERVAL_SEC}s. Ctrl+C để dừng.")
        while True:
            img = render_stats_image()
            t0 = time.time()
            await self.send_image(img)
            print(f"  -> đã gửi & refresh, mất {time.time() - t0:.1f}s")

            await asyncio.sleep(UPDATE_INTERVAL_SEC)


def load_cached_address():
    """Đọc địa chỉ raw từ file txt"""
    if os.path.exists(CACHE_TXT):
        try:
            with open(CACHE_TXT, "r", encoding="utf-8") as f:
                addr = f.read().strip()
                return addr if addr else None
        except Exception as e:
            print(f"Không đọc được file cấu hình địa chỉ: {e}")
    return None

def save_cached_address(address):
    """Ghi đè địa chỉ raw vào file txt"""
    try:
        with open(CACHE_TXT, "w", encoding="utf-8") as f:
            f.write(address.strip())
        print(f"--> Đã lưu địa chỉ [{address}] vào file {CACHE_TXT}")
    except Exception as e:
        print(f"Không thể ghi địa chỉ vào file txt: {e}")
        
        

async def scan_and_connect() -> BleakClient:
    cached_addr = load_cached_address()
    target_name = None
    target_address = None
    if SCAN_MODE == 0:
        target_name = '[FIXED]'
        target_address = cached_addr
    else:
        print("Đang scan thiết bị Bluetooth...")
        devices = await BleakScanner.discover(timeout=20.0)
        for i, d in enumerate(devices):
            print(f"  [{i}] {d.name or '(không tên)'}  {d.address}")
        if DEVICE_NAME_FILTER:
            devices = [d for d in devices if d.name and DEVICE_NAME_FILTER in d.name]

        if not devices:
            print("Không tìm thấy thiết bị nào. Kiểm tra Bluetooth đã bật và màn hình đang ở chế độ chờ kết nối.")
            sys.exit(1)
        print("Chọn thiết bị:")
        if len(devices) == 1:
            target = devices[0]
        else:
            choice = input("Nhập số thứ tự: ").strip()
            target = devices[int(choice)]
        target_name = target.name
        target_address = target.address
    if target_address is None:
        raise Exception("Địa chỉ thiết bị không hợp lệ")
    if target_address != cached_addr:
        save_cached_address(target_address)
    client = BleakClient(target_address)     
    print(f"Đang kết nối: {target_name} ({target_address})")
    await client.connect()
    print(f"Đã kết nối: {target_name} ({target_address})")
    return client


        
async def main():
    if DEBUG_SAVE_ONLY:
        img = render_stats_image()
        img.save(DEBUG_OUTPUT_PATH)
        print(f"Đã lưu ảnh debug tại: {DEBUG_OUTPUT_PATH}")
        print("Mở file này lên xem trực tiếp (đúng pixel sẽ gửi xuống màn hình, "
              "ảnh nhỏ 122x250 nên có thể cần zoom).")
        return

    client = await scan_and_connect()
    epd = EpdDevice(client)

    try:
        try:
            version_data = await client.read_gatt_char(VERSION_CHAR_UUID)
            print(f"Firmware version: 0x{version_data[0]:02x}")
        except Exception:
            print("Không đọc được version, bỏ qua.")

        # Đọc MTU thật (chỉ dùng nếu AUTO_DETECT_MTU=True, mặc định tắt vì
        # bleak trên Windows hay báo sai giá trị này - xem giải thích ở đầu file).
        if AUTO_DETECT_MTU:
            try:
                negotiated_mtu = client.mtu_size
                real_chunk_payload = negotiated_mtu - 3 - 1
                epd.chunk_size = max(20, real_chunk_payload - 2)
                print(f"MTU thực tế (theo bleak): {negotiated_mtu}, dùng chunk_size = {epd.chunk_size}")
            except Exception as e:
                print(f"Không đọc được MTU thực tế ({e}), dùng SAFE_CHUNK_SIZE = {epd.chunk_size}")
        # else:
        #     print(f"Dùng chunk_size cố định an toàn = {epd.chunk_size} (AUTO_DETECT_MTU=False)")

        if EPD_PINS_HEX:
            await epd.write(EpdCmd.SET_PINS, hex2bytes(EPD_PINS_HEX))
        if EPD_DRIVER_HEX:
            await epd.write(EpdCmd.INIT, hex2bytes(EPD_DRIVER_HEX))

        if MODE == 1:
            await epd.sync_metrics()
        elif MODE == 2:
            await epd.sync_time(1)
        elif MODE == 3:
            await epd.sync_time(2)
        
        

    except KeyboardInterrupt:
        print("\nDừng theo yêu cầu người dùng.")
    finally:
        await client.disconnect()
        print("Đã ngắt kết nối.")


if __name__ == "__main__":
    asyncio.run(main())

# ============== DEBUG NẾU ẢNH SAI ==============
# Định dạng bitmap (threeColor, 2 plane) đã khớp 100% với code gốc.
# Nếu vẫn gặp vấn đề khi chạy thực tế trên phần cứng:
# 1. Ảnh chỉ hiện 1 phần / lỗi giữa chừng -> giảm SAFE_CHUNK_SIZE (thử nhỏ hơn 242).
# 2. Màu/driver sai hoàn toàn -> kiểm tra lại đúng driver/màu trên dropdown 颜色模式
#    của web app khi kết nối (đã xác nhận máy bạn là threeColor, không phải
#    blackWhiteColor) - nếu sau này đổi sang driver/màn khác, nhớ đổi lại
#    pipeline đóng gói cho khớp.
# 3. Mất kết nối giữa chừng khi gửi nhiều chunk liên tiếp không response -> giảm
#    INTERLEAVED_COUNT xuống thấp hơn 50.