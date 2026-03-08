多旋翼下降速度控制逻辑



1. 下降速度由两个PILOT_DN_ALT_HI和PILOT_DN_ALT_LO高度阈值分割成3个阶段：分别是高速下降阶段，减速阶段，低速阶段。当前高度>PILOT_DN_ALT_HI时，使用PILOT_SPEED_DN作为最大下降速度，当前高度<PILOT_DN_ALT_LO时使用PILOT_SPD_DN_LOW作为最大下降速度，当前高度介于PILOT_DN_ALT_LO和PILOT_DN_ALT_HI之间时，下降速度根据高度不同，线性的从PILOT_SPEED_DN降低到PILOT_SPD_DN_LOW

2. 由于气压计计算的高度会产生漂移，因此当rangefinder测量数值有效时，总是优先使用rangefinder的数据来作为高度值用于计算当前允许的最大下降速度，当rangefinder数据无效时，使用EK3融合的相对高度来计算最大下降速度

3. rangefinder最大量程与PILOT_DN_ALT_LO和PILOT_DN_ALT_HI的关系为 PILOT_DN_ALT_LO<rangefinder最大量程<PILOT_DN_ALT_HI，因此下降减速过程中会存在先使用EK3相对高度计算，降速到一定高度rangefinder生效，再使用rangefinder计算的切换

4. 最大下降速度计算，分为3种情况：

   a. 起飞后，又降落到原地，比如说从楼顶起飞，又返回楼顶降落，这种情况按以上定义的来实现

   b. 起飞后，降落到比原地矮的地方，比如说从楼顶起飞降落到楼底

   ​    相对高度>PILOT_DN_ALT_HI时，按最大PILOT_SPEED_DN下降，相对高度<70%*rangefinder最大量程，而rangefinder还是无效时，说明此时降落的地方比起飞的地方矮，下降速度钳位到70%*rangefinder对应高度的下降速度，不再降低，直到rangefinder数值重新有效，则根据rangefinder的高度来计算实时下降速度

   c. 起飞后，降落到比原地高的地方，比如说从楼底起飞降落到楼顶

   ​    当相对高度>PILOT_DN_ALT_HI时，使用PILOT_SPEED_DN作为下降速度，但是当相对高度>rangefinder最大量程，而rangefinder又能测量到有效数值时，说明此时降落的地方高于起飞位置，此时需要根据rangefinder的测量高度，将下降速度根据rangefinder的最大高度时为PILOT_SPEED_DN线性减小到PILOT_SPD_DN_LOW



基于以上说明，帮我实现这个降落速度控制的逻辑