import cv2
from pypylon import pylon

# ---------- CAMERA SETUP ----------
tl_factory = pylon.TlFactory.GetInstance()
devices = tl_factory.EnumerateDevices()
if not devices:
    raise RuntimeError("No Basler camera found")

camera = pylon.InstantCamera(tl_factory.CreateDevice(devices[0]))
camera.Open()
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

converter = pylon.ImageFormatConverter()
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

# ---------- DISPLAY LOOP ----------
try:
    while camera.IsGrabbing():
        grab = camera.RetrieveResult(2000, pylon.TimeoutHandling_ThrowException)
        frame = converter.Convert(grab).GetArray()
        grab.Release()

        cv2.imshow("Camera View", frame)
        if cv2.waitKey(1) & 0xFF in (27, ord('q')):
            break

finally:
    camera.StopGrabbing()
    camera.Close()
    cv2.destroyAllWindows()
