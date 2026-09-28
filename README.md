# Automated Delivery Billing System Based on 8x8 LiDAR Distance Measurement

An automated parcel measurement station built on the **UNIHIKER K10**. A parcel is placed on the
platform, and the system measures its **length, width and height** with an 8x8 matrix LiDAR ToF
sensor mounted above, reads its **actual weight** from a load cell below, then computes
**volumetric weight** and **reference billable weight** and shows the shipping-cost estimate
through a built-in web page.

No manual weighing, no ruler, no paperwork — one placement, all the numbers.

---

## The Problem

At a courier drop-off point during peak season, staff repeat the same loop for every parcel:

1. Put the parcel on an electronic scale and read the weight.
2. Measure length, width and height separately with a ruler.
3. Decide the billable weight from dimensions and weight.

It is slow, and manual readings are easy to get wrong.

## The Idea

Could all of that be folded into a single device? The operator places the parcel on the detection
platform, and the device obtains the dimensions and the weight on its own, then uses those results
for volumetric weight and a shipping-cost estimate.

## How It Works

The sensor above the platform does not return a single distance — it returns a grid of **64
distance measurement zones** that together describe the parcel's position and contour. From that
grid the system derives length, width and height.

The pipeline is:

| Stage | What happens |
| --- | --- |
| **Empty-scene learning** | On boot the system samples the bare platform and stores an 8x8 background depth map. This is the reference for "nothing is here". |
| **Depth filtering** | Noise in the raw ToF readings is suppressed with a median filter over recent frames. |
| **Background subtraction** | Subtracting the stored background isolates the zones covered by the parcel — and only those. |
| **Boundary extraction** | The occupied zones give the parcel's footprint edges in the sensor's field of view. |
| **Triangular projection** | Combining edge positions, the sensor's field of view, and the measured depth converts everything into real-world length, width and height. |
| **Weight sensing** | The load cell under the platform supplies the actual weight. |
| **Billing** | Dimensions and weight combine into volumetric weight and reference billable weight; results and cost estimates render on the web page. |

The key insight from this build: an 8x8 ToF array is not about reading 64 distance values. It is
about turning those discrete readings into meaningful dimensional information through empty-field
learning, depth filtering, background subtraction, boundary extraction and triangular projection.

---

## Hardware

| Part | Product link | Qty |
| --- | --- | --- |
| UNIHIKER K10 | [dfrobot.com/product-2904](https://www.dfrobot.com/product-2904.html) | 1 |
| SEN0628 — Matrix Laser Distance Measurement Sensor (8x8) | [dfrobot.com.cn/goods-4243](https://www.dfrobot.com.cn/goods-4243.html) | 1 |
| HX711 I2C weight sensor / load cell | — | 1 |

## Repository Contents

```
.
├── Automated Delivery Billing System Based on 8x8 Lidar Distance Measurement.md   # full project write-up
├── Images_attachments/
│   ├── K10_SEN0628_HX711_All_English_Inch_Lb.ino       # complete Arduino firmware
│   ├── Parcel_Station_International_US_EN_v2.html      # web UI for the K10
│   ├── DFRobot_MatrixLidar.zip                         # 8x8 ToF sensor library
│   ├── DFRobot_HX711_I2C-master.zip                    # weight sensor library
│   └── image*.png, *.jpg                               # build photos and screenshots
└── README.md
```

## Demo

[![Watch the demo](https://img.youtube.com/vi/JUHfYTu8ryg/maxresdefault.jpg)](https://youtu.be/JUHfYTu8ryg)
The demo video is hosted on YouTube rather than committed here, to keep the repository small.

## Getting Started

1. Install the two libraries from `Images_attachments/` into your Arduino libraries folder
   (`DFRobot_MatrixLidar.zip`, `DFRobot_HX711_I2C-master.zip`).
2. Open `K10_SEN0628_HX711_All_English_Inch_Lb.ino` in the Arduino IDE with the UNIHIKER K10 board
   selected.
3. Upload the firmware.
4. Upload `Parcel_Station_International_US_EN_v2.html` to the K10 so the web interface is served.
5. Power the device and keep the platform **empty** during startup — the boot sequence learns the
   empty background and cannot measure correctly without it.

> **Note on units:** the firmware in this repository is configured for **inches and pounds**
> (hence `Inch_Lb` in the filename). If you need metric output, adjust the conversion constants in
> the sketch.

## Future Improvements

- Camera-based visual recognition for parcel shape.
- Higher-resolution depth sensors for finer edge resolution.
- Integration with real logistics quotation interfaces, moving the build closer to a practical
  intelligent shipping terminal.

## Acknowledgements

Sensor libraries are the official DFRobot releases. Built and documented for the DFRobot maker
community.

## License

See [LICENSE](LICENSE).
