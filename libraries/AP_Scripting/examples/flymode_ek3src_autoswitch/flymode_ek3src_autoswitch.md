# 飞行模式和EKF3数据源自动切换lua脚本功能定义

本脚本需要实现AltHold和Loiter模式之间的自动切换，以及在loiter模式时，EK3的数据源在GPS和光流之间的自动切换。

切换逻辑定义如下：

1. 当飞机飞行模式为模式6的时候，是手动althold模式，此时固定使用EK3_SRC3作为数据源。当飞机飞行模式为模式4的时候，是自动切换模式，在AltHold和Loiter这两个模式之前根据之后的规则动态切换模式。

2. 从手动模式切换到自动模式时，先维持在AltHold+EK3_SRC3的状态，满足下面几点的切换状态时，才实际切换到对应的Loiter+SRC

3. **`EK3_SRC1_POSZ` 与 `EK3_SRC2_POSZ` 固定为测距仪（RangeFinder）**，脚本不在气压/GPS/测距之间切换高度源；超过测距量程时的融合行为由飞控 EKF/参数（如 `RNGFNDx`、EKF 相关项）决定。

4. EK3默认SRC1是使用GPS的配置，SRC2是使用光流的配置。EK3的数据源使用光流还是GPS，使用投票机制来切换。当光流满足置信度和创新值阈值时，投票给光流。当GPS满足 **3D 定位**、**卫星数 >15（至少 16 颗）**、**位置精度 p_acc = sqrt(hacc²+vacc²) < 1 m**、以及 **SCR_USER1 速度精度门限** 时投票给 GPS。**当光流（含测距）与 GPS 同时满足各自阈值时，始终优先使用光流数据源（SRC2）**；投票主要用于在「仅光流好」「仅 GPS 好」之间收敛，避免抖动。

5. 在自动AltHold/Loiter模式下时，持续监测rangefinder，光流和GPS的状态，切换这两个模式模式

   - 当前在Loiter模式时，如果rangefinder+光流满足条件，优先切换使用rangefinder+光流作为定点数据源。如果rangefinder+光流不满足条件时，GPS满足条件，切换到使用GPS作为数据源。
   - 当前在Loiter模式时，如果rangefinder+光流+GPS都不满足条件，切换到AltHold模式
   - 当前在AltHold模式时，如果如果rangefinder+光流满足条件，数据源切换到使用rangefinder+光流作为定点数据源，并将模式切换到loiter
   - 当前在AltHold模式时，如果如果GPS满足条件，数据源切换到使用GPS作为定点数据源，并将模式切换到loiter

6. 在Loiter模式下时，数据源由第4条投票与实时条件共同决定；**光流与 GPS 同时满足阈值时固定优先光流（SRC2）**。当按阈值判断光流与 GPS 均不可用时，退化到 AltHold 模式（与第5条一致）。

7. 当手动设置的飞行模式处于AltHold的时候，不参与以上的切换，并且切换到EK3_SRC3将EK3_SRC3配置成

   --   EK3_SRC3_POSXY = 0 (None)

   --   EK3_SRC3_POSZ  = 1 (Baro)

   --   EK3_SRC3_VELXY = 0 (None)

   --   EK3_SRC3_VELZ  = 0 (None)

   --   EK3_SRC3_YAW  = 1 (Compass)

8. 当使用GPS作为数据源的时候，一旦发现GPS不满足条件，优先看光流是否符合条件，光流如果还不满足条件时才自动切换到althold

---

## 实现与排错说明

脚本实现细节、Lua 绑定常见错误、`gcs:send_text` 长度与 hAcc/vAcc 相关说明见：**[脚本排错与绑定注意事项.md](./脚本排错与绑定注意事项.md)**。
