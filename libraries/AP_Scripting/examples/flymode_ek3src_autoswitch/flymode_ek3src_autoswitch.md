# 飞行模式和EKF3数据源自动切换lua脚本功能定义

本脚本需要实现AltHold和Loiter模式之间的自动切换，以及在loiter模式时，EK3的数据源在GPS和光流之间的自动切换。

切换逻辑定义如下：

1. 开机默认是使用rangefinder作为高度源，以rangefinder量程*0.8为分界线，加一个迟滞比较器，迟滞宽度为0.1*rangefinder。

   - 使用rangefinder高度超过(0.8+0.1)*rangefinder_max时，从rangdfinder切换到baro/GPS*。
   - *使用baro/GPS高度小于(0.8-0.1)*rangefinder_max时切换回rangefinder。
   - 高空(高度>(0.8+0.1)*rangefinder_max)时，高度源使用baro还是GPS，取决于当前的飞行模式和是否使用GPS作为数据源。当高空处于AltHold时，一直使用baro作为高度源，当高空处于Loiter时，如果GPS信号好可以使用GPS数据源时，才使用GPS作为高度源
   - 使用rangefinder来判断当前是否是在高空状态，当rangefinder一直超量程时，即使飞行已经低于起飞点，也需要认为当前高度是高空(从楼顶飞到楼底的情况)

2. EK3默认SRC1是使用GPS的配置，SRC2是使用光流的配置。EK3的数据源使用光流还是GPS，使用投票机制来切换。当光流满足置信度和创新值阈值时，投票给光流。当GPS满足速度精度，位置精度，卫星颗数的要求时投票给GPS。**当光流（含测距）与 GPS 同时满足各自阈值时，始终优先使用光流数据源（SRC2）**；投票主要用于在「仅光流好」「仅 GPS 好」之间收敛，避免抖动。

3. 在自动AltHold/Loiter模式下时，持续监测rangefinder，光流和GPS的状态，切换这两个模式模式

   - 当前在Loiter模式时，如果rangefinder+光流满足条件，优先切换使用rangefinder+光流作为定点数据源。如果rangefinder+光流不满足条件时，GPS满足条件，切换到使用GPS作为数据源。
   - 当前在Loiter模式时，如果rangefinder+光流+GPS都不满足条件，切换到AltHold模式
   - 当前在AltHold模式时，如果如果rangefinder+光流满足条件，数据源切换到使用rangefinder+光流作为定点数据源，并将模式切换到loiter
   - 当前在AltHold模式时，如果如果GPS满足条件，数据源切换到使用GPS作为定点数据源，并将模式切换到loiter

4. 在Loiter模式下时，数据源由第2条投票与实时条件共同决定；**光流与 GPS 同时满足阈值时固定优先光流（SRC2）**。当按阈值判断光流与 GPS 均不可用时，退化到 AltHold 模式（与第3条一致）。

5. 当手动设置的飞行模式处于AltHold的时候，不参与以上的切换，并且切换到EK3_SRC3将EK3_SRC3配置成

   --   EK3_SRC3_POSXY = 0 (None)

   --   EK3_SRC3_POSZ  = 1 (Baro)

   --   EK3_SRC3_VELXY = 0 (None)

   --   EK3_SRC3_VELZ  = 0 (None)

   --   EK3_SRC3_YAW  = 1 (Compass)

6. 当手动设置飞行模式处于Loiter模式的时候，才参与以上1-4点的切换

7. 当使用GPS作为数据源的时候，一旦发现GPS不满足条件，优先看光流是否符合条件，光流如果还不满足条件时才自动切换到althold

   