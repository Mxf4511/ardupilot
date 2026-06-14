--[[
   ESC 老化测试脚本
   控制四旋翼 4 个电机同步进行油门阶跃老化测试
   
   电机通道：3, 4, 5, 6（对应 SERVO3=Motor4, SERVO4=Motor1, SERVO5=Motor2, SERVO6=Motor3）
   油门映射：pwm = 1000 + thr% × 10（0%=1000μs, 100%=2000μs）
   
   使用前提：
   - 务必拆除螺旋桨或使用测试台架
   - ESC 已正确校准
   
   参数说明：
   - ETEST_ENABLE: 使能控制（0=停止, 1=开始）
   - ETEST_THR: 油门百分比（0-100%）
   - ETEST_DUR: 持续时间（秒）
   - ETEST_LOOP: 循环次数（负数=无限, 0或1=单次）
   - ETEST_NOISE: 随机噪声幅度（0-30%），0=无噪声
--]]

-- luacheck: only 0

-- 参数表配置
local PARAM_TABLE_KEY = 137
local PARAM_TABLE_PREFIX = 'ETEST_'

-- 添加参数的辅助函数
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
           string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- 创建参数表
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10),
       'could not add param table')

-- 定义参数
local ETEST_ENABLE = bind_add_param('ENABLE', 1, 0)
local ETEST_THR    = bind_add_param('THR',    2, 50)
local ETEST_DUR    = bind_add_param('DUR',    3, 10)
local ETEST_LOOP   = bind_add_param('LOOP',   4, 1)
local ETEST_NOISE  = bind_add_param('NOISE',  5, 0)  -- 随机噪声幅度（0-30%）

-- 保存原始参数值（用于恢复）
local orig_fs_thr_enable = nil
local orig_fs_gcs_enable = nil
local orig_fs_ekf_action = nil
local orig_arming_check = nil

-- 电机通道硬编码（0-indexed: 2,3,4,5 → CH3=Motor4, CH4=Motor1, CH5=Motor2, CH6=Motor3）
local MOTOR_CHANS = {2, 3, 4, 5}

-- PWM 范围常量
local PWM_MIN = 1000
local PWM_MAX = 2000

-- 状态机状态定义
local STATE_IDLE         = 0
local STATE_THROTTLE_UP  = 1
local STATE_THROTTLE_OFF = 2

-- 当前状态
local state = STATE_IDLE

-- 当前阶段开始时间（毫秒）
local cycle_start_ms = 0

-- 剩余循环次数（运行时副本）
local loop_count = 0

-- 停止后保持最低油门输出（防止飞控接管）
local motors_stopped = false

-- 上电安全保护：立即将使能清零，防止异常启动
ETEST_ENABLE:set_and_save(0)

-- 调试信息：打印当前状态
local function debug_info()
    local armed = arming:is_armed()
    local e_stop = SRV_Channels:get_emergency_stop()
    local safe = SRV_Channels:get_safety_state()
    gcs:send_text(6, string.format('ESC_AGE DEBUG: ARMED=%s E-STOP=%s SAFE=%s',
        tostring(armed), tostring(e_stop), tostring(safe)))
    if not armed then
        gcs:send_text(6, 'ESC_AGE: WARNING: Motors may not spin when DISARMED!')
        gcs:send_text(6, 'ESC_AGE: Use "arm force" to arm the vehicle.')
    end
end

-- 设置所有电机 PWM 输出
local function set_motors_pwm(pwm)
    for _, ch in ipairs(MOTOR_CHANS) do
        SRV_Channels:set_output_pwm_chan_timeout(ch, pwm, 200)
    end
end

-- 将油门百分比转换为 PWM 值
local function thr_to_pwm(thr_pct)
    -- 加入随机噪声
    local noise = ETEST_NOISE:get()
    if noise > 0 then
        -- 生成 -noise ~ +noise 的随机偏移
        local offset = (math.random() * 2 - 1) * noise
        thr_pct = thr_pct + offset
        thr_pct = math.max(0, math.min(100, thr_pct))  -- 限制在 0-100%
    end
    return math.floor(PWM_MIN + (PWM_MAX - PWM_MIN) * thr_pct / 100)
end

-- 停止所有电机
local function motors_stop()
    set_motors_pwm(PWM_MIN)
end

-- 发送状态消息到 GCS
local function send_status(msg)
    gcs:send_text(6, 'ESC_AGE: ' .. msg)
end

-- 禁用 failsafe（仅用于测试）
local function disable_failsafe()
    -- 获取参数对象（使用 pcall 防止参数不存在导致崩溃）
    local ok1, p1 = pcall(function() return Parameter('FS_THR_ENABLE') end)
    local ok2, p2 = pcall(function() return Parameter('FS_GCS_ENABLE') end)
    local ok3, p3 = pcall(function() return Parameter('FS_EKF_ACTION') end)
    local ok4, p4 = pcall(function() return Parameter('ARMING_CHECK') end)
    
    -- 保存原始值并设置新值
    if ok1 and p1 then
        orig_fs_thr_enable = p1
        p1:set_and_save(0)  -- 禁用油门 failsafe
    end
    if ok2 and p2 then
        orig_fs_gcs_enable = p2
        p2:set_and_save(0)  -- 禁用 GCS failsafe
    end
    if ok3 and p3 then
        orig_fs_ekf_action = p3
        p3:set_and_save(0)  -- 禁用 EKF failsafe
    end
    if ok4 and p4 then
        orig_arming_check = p4
        p4:set_and_save(0)  -- 禁用解锁前检查
    end
    send_status('Failsafe disabled for testing')
end

-- 恢复 failsafe 设置
local function restore_failsafe()
    if orig_fs_thr_enable then
        orig_fs_thr_enable:set_and_save(orig_fs_thr_enable:get())
    end
    if orig_fs_gcs_enable then
        orig_fs_gcs_enable:set_and_save(orig_fs_gcs_enable:get())
    end
    if orig_fs_ekf_action then
        orig_fs_ekf_action:set_and_save(orig_fs_ekf_action:get())
    end
    if orig_arming_check then
        orig_arming_check:set_and_save(orig_arming_check:get())
    end
    send_status('Failsafe settings restored')
end

-- 状态机主更新函数
function update()
    local now = millis()

    if state == STATE_IDLE then
        -- 检查是否启动测试
        if ETEST_ENABLE:get() == 1 then
            -- 禁用 failsafe 防止自动上锁
            disable_failsafe()
            
            -- 检查是否已解锁，未解锁则尝试解锁
            if not arming:is_armed() then
                send_status('Arming vehicle...')
                if not arming:arm_force() then
                    send_status('ERROR: Arm failed! Check PreArm errors.')
                    restore_failsafe()
                    ETEST_ENABLE:set_and_save(0)
                    return update, 100
                end
                send_status('Vehicle armed successfully')
            end

            -- 读取测试参数
            local thr = ETEST_THR:get()
            local dur = ETEST_DUR:get()
            local loop = ETEST_LOOP:get()

            -- 参数合法性检查
            if thr < 0 or thr > 100 then
                send_status('ERROR: THR must be 0-100')
                restore_failsafe()
                ETEST_ENABLE:set_and_save(0)
                return update, 100
            end

            if dur <= 0 then
                send_status('ERROR: DUR must be > 0')
                restore_failsafe()
                ETEST_ENABLE:set_and_save(0)
                return update, 100
            end

            if ETEST_NOISE:get() < 0 or ETEST_NOISE:get() > 30 then
                send_status('ERROR: NOISE must be 0-30')
                restore_failsafe()
                ETEST_ENABLE:set_and_save(0)
                return update, 100
            end

            -- 初始化循环计数
            if loop < 0 then
                loop_count = -1  -- 无限循环标记
            else
                loop_count = math.max(loop, 1)  -- 0和1都是单次
            end

            -- 进入油门阶跃状态
            state = STATE_THROTTLE_UP
            cycle_start_ms = now
            motors_stopped = false  -- 重置停止标志

            local pwm = thr_to_pwm(thr)
            set_motors_pwm(pwm)
            debug_info()
            local noise = ETEST_NOISE:get()
            send_status(string.format('START: THR=%d%% PWM=%d DUR=%ds LOOP=%s NOISE=%d%%',
                thr, pwm, dur, loop < 0 and 'INF' or tostring(loop), noise))
        end

    elseif state == STATE_THROTTLE_UP then
        -- 持续时间到期，进入油门归零状态
        local dur_ms = ETEST_DUR:get() * 1000
        if (now - cycle_start_ms) >= dur_ms then
            state = STATE_THROTTLE_OFF
            cycle_start_ms = now

            motors_stop()
            send_status('THROTTLE_OFF: 2s cooldown')
        end

    elseif state == STATE_THROTTLE_OFF then
        -- 2秒冷却时间到期，检查循环次数
        if (now - cycle_start_ms) >= 2000 then
            -- 检查是否还有循环次数
            if loop_count < 0 then
                -- 无限循环，继续
                state = STATE_THROTTLE_UP
                cycle_start_ms = now

                local pwm = thr_to_pwm(ETEST_THR:get())
                set_motors_pwm(pwm)
                send_status(string.format('LOOP: THR=%d%% PWM=%d', ETEST_THR:get(), pwm))
            elseif loop_count > 1 then
                -- 还有剩余次数，递减并继续
                loop_count = loop_count - 1
                state = STATE_THROTTLE_UP
                cycle_start_ms = now

                local pwm = thr_to_pwm(ETEST_THR:get())
                set_motors_pwm(pwm)
                send_status(string.format('LOOP: THR=%d%% PWM=%d REMAIN=%d', ETEST_THR:get(), pwm, loop_count))
            else
                -- 测试结束
                state = STATE_IDLE
                motors_stopped = true  -- 保持最低油门输出

                motors_stop()
                -- 尝试上锁（如果失败也不影响，override 会持续保持最低油门）
                if arming:is_armed() then
                    arming:disarm()
                end
                restore_failsafe()
                ETEST_ENABLE:set_and_save(0)
                send_status('COMPLETE: Test finished, ENABLE cleared')
            end
        end
    end

    -- 检查是否被外部停止（在非 IDLE 状态下检查）
    if state ~= STATE_IDLE and ETEST_ENABLE:get() == 0 then
        state = STATE_IDLE
        motors_stopped = true  -- 保持最低油门输出

        motors_stop()
        -- 尝试上锁（如果失败也不影响，override 会持续保持最低油门）
        if arming:is_armed() then
            arming:disarm()
        end
        restore_failsafe()
        send_status('STOPPED: Emergency stop')
    end

    -- 持续刷新 override timeout
    if state == STATE_THROTTLE_UP then
        -- 测试中：持续输出目标油门
        local pwm = thr_to_pwm(ETEST_THR:get())
        set_motors_pwm(pwm)
    elseif state ~= STATE_IDLE or motors_stopped then
        -- IDLE 或已停止：持续输出最低油门，保持 override 防止飞控接管
        set_motors_pwm(PWM_MIN)
    end

    return update, 10  -- 10ms 调用周期（100Hz）
end

-- 脚本启动
debug_info()
send_status('Script loaded. Set ETEST_ENABLE=1 to start.')
return update, 1000  -- 1秒后启动
