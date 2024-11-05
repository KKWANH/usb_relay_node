import hid

# USB 릴레이의 Vendor ID와 Product ID를 설정합니다.
VENDOR_ID = 0x16c0
PRODUCT_ID = 0x05df

# HID 장치 열기
try:
    device = hid.Device(vid=VENDOR_ID, pid=PRODUCT_ID)
    print(f"Device manufacturer: {device.manufacturer}")
    print(f"Product: {device.product}")
except IOError as ex:
    print(f"장치를 열 수 없습니다: {ex}")
    exit(1)

# 릴레이 제어 명령어 예제
# 첫 번째 릴레이를 켜는 명령어를 bytearray 형식으로 전송합니다.
relay_on_command = bytearray([0x00, 0x01])  # 예시 명령어

try:
    device.write(relay_on_command)
    print("릴레이가 성공적으로 제어되었습니다.")
except Exception as e:
    print(f"릴레이 제어 중 오류 발생: {e}")

# 장치 닫기
device.close()
