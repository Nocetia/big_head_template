# 在下方板级循环中调用，使用当前 _board 及其 config.hpp 内容 _resources。
# none 转为空资源；其他名称必须由该板导出。
function(config_resource out kind key)
    string(JSON name GET "${_json}" boards ${_board} ${key})
    string(JSON type TYPE "${_json}" boards ${_board} ${key})
    if(NOT type STREQUAL "STRING" OR NOT name MATCHES "^[a-z][a-z0-9_]*$")
        message(FATAL_ERROR "boards.${_board}.${key}: expected a resource name")
    endif()
    if(NOT name STREQUAL "none" AND
       NOT _resources MATCHES "inline constexpr ${kind}::config ${name}\\{")
        message(FATAL_ERROR "boards.${_board}.${key}: unknown ${kind} resource '${name}'")
    endif()
    set(value "bsp::peripherals::${name}")
    if(name STREQUAL "none")
        set(value "bsp::${kind}::config{}")
    endif()
    if(kind STREQUAL "spi")
        set(value "bsp::spi::handle::${name}")
    endif()
    set(${out} "${value}" PARENT_SCOPE)
endfunction()

# 两块板都写入生成头；当前 BOARD 的后端选择同时导出给构建系统。
set(_boards "")
foreach(_board IN ITEMS h723_mc02 f407_cboard)
    file(READ "${_config_dir}/../boards/wrapper/${_board}/include/config.hpp" _resources)
    if(_board STREQUAL "h723_mc02")
        set(_condition "#if defined(STM32H723xx)")
        set(_can_names "^(none|fdcan[123])$")
    else()
        set(_condition "#elif defined(STM32F407xx)")
        set(_can_names "^(none|can[12])$")
    endif()
    string(JSON _dm_bus GET "${_json}" boards ${_board} dmimu_can)
    if(NOT _dm_bus MATCHES "${_can_names}")
        message(FATAL_ERROR "boards.${_board}.dmimu_can: unsupported CAN resource")
    endif()
    string(JSON _ahrs_device GET "${_json}" boards ${_board} ahrs_device)
    if(_ahrs_device STREQUAL "none")
        set(_ahrs_backend 0)
    elseif(_ahrs_device STREQUAL "bmi088")
        set(_ahrs_backend 1)
    elseif(_ahrs_device STREQUAL "dmimu")
        set(_ahrs_backend 2)
        if(_dm_bus STREQUAL "none")
            message(FATAL_ERROR "boards.${_board}: DMIMU AHRS requires dmimu_can")
        endif()
    else()
        message(FATAL_ERROR "boards.${_board}.ahrs_device: expected none/bmi088/dmimu")
    endif()
    config_bool(_heater_enabled boards ${_board} bmi088_heater_enabled)
    config_float(_thermal_target 20 60 boards ${_board} bmi088_thermal target)
    foreach(_duty IN ITEMS boost reboost approach_start approach_end hold_max)
        config_float(_thermal_${_duty} 0 1 boards ${_board} bmi088_thermal ${_duty})
    endforeach()
    foreach(_gain IN ITEMS kp ki kd)
        config_float(_thermal_${_gain} 0 10000 boards ${_board} bmi088_thermal ${_gain})
    endforeach()
    config_float(_thermal_max_out 0 999 boards ${_board} bmi088_thermal max_out)
    config_resource(_rfid_uart usart rfid_uart)
    config_resource(_buzzer_timer pwm buzzer_timer)
    config_integer(_buzzer_channel 1 4 boards ${_board} buzzer_channel)
    if(NOT _buzzer_timer STREQUAL "bsp::pwm::config{}")
        string(JSON _timer_name GET "${_json}" boards ${_board} buzzer_timer)
        string(REGEX MATCH "pwm::config ${_timer_name}\\{[^}]+, (0x[0-9A-Fa-f]+)U" _timer_match "${_resources}")
        if(NOT _timer_match)
            message(FATAL_ERROR "Cannot find buzzer timer channel mask")
        endif()
        math(EXPR _channel_available "${CMAKE_MATCH_1} & (1 << (${_buzzer_channel} - 1))")
        if(NOT _channel_available)
            message(FATAL_ERROR "Buzzer timer channel is not configured in CubeMX")
        endif()
        if((_board STREQUAL "h723_mc02" AND _timer_name STREQUAL "tim3") OR
           (_board STREQUAL "f407_cboard" AND _timer_name STREQUAL "tim10" AND _heater_enabled))
            message(FATAL_ERROR "Buzzer must not share the BMI088 heater timer")
        endif()
    endif()
    config_resource(_referee_uart usart referee_uart)
    foreach(_remote IN ITEMS dr16 vt03 ps2)
        config_bool(_enable_${_remote} boards ${_board} remoter_${_remote}_enabled)
        if(_enable_${_remote})
            set(_remote_${_remote} 1)
        else()
            set(_remote_${_remote} 0)
        endif()
    endforeach()
    config_resource(_ui_uart usart ui_uart)
    config_resource(_dr16_uart usart dr16_uart)
    config_resource(_vt03_uart usart vt03_uart)
    config_resource(_ps2_cs gpio ps2_cs)
    config_resource(_ps2_spi spi ps2_spi)

    if(_enable_dr16 AND _dr16_uart STREQUAL "bsp::usart::config{}")
        message(FATAL_ERROR "DR16 requires a UART")
    endif()
    if(_enable_vt03 AND _vt03_uart STREQUAL "bsp::usart::config{}")
        message(FATAL_ERROR "VT03 requires a UART")
    endif()
    if(_enable_dr16 AND _enable_vt03 AND _dr16_uart STREQUAL _vt03_uart)
        message(FATAL_ERROR "DR16 and VT03 require different UARTs")
    endif()
    if((_enable_dr16 AND _referee_uart STREQUAL _dr16_uart) OR
       (_enable_vt03 AND _referee_uart STREQUAL _vt03_uart))
        message(FATAL_ERROR "Referee and remoter must not share RX ownership")
    endif()
    if(_enable_ps2)
        if(_ps2_cs STREQUAL "bsp::gpio::config{}")
            message(FATAL_ERROR "PS2 requires CS")
        endif()
        if(_ps2_spi STREQUAL "bsp::spi::handle::none")
            message(FATAL_ERROR "PS2 requires a hardware SPI")
        endif()
    endif()
    if(NOT _rfid_uart STREQUAL "bsp::usart::config{}")
        if(_rfid_uart STREQUAL _referee_uart OR _rfid_uart STREQUAL _ui_uart OR
           (_enable_dr16 AND _rfid_uart STREQUAL _dr16_uart) OR
           (_enable_vt03 AND _rfid_uart STREQUAL _vt03_uart))
            message(FATAL_ERROR "RFID UART must not share UI/referee/remoter ownership")
        endif()
        string(JSON _uart_name GET "${_json}" boards ${_board} rfid_uart)
        if(NOT _resources MATCHES "usart::config ${_uart_name}\\{[^}]+true")
            message(FATAL_ERROR "RFID UART requires RX DMA")
        endif()
    endif()
    if(_rfid_uart STREQUAL "bsp::usart::config{}" OR _buzzer_timer STREQUAL "bsp::pwm::config{}")
        message(FATAL_ERROR "RFID service requires boards.${_board}.rfid_uart and buzzer_timer")
    endif()
    # Export the same validated choices to CMake for the selected board.
    if(_board STREQUAL BOARD)
        set(USER_AHRS_DEVICE "${_ahrs_device}")
        set(USER_REMOTER_DR16 ${_remote_dr16})
        set(USER_REMOTER_VT03 ${_remote_vt03})
        set(USER_REMOTER_PS2 ${_remote_ps2})
    endif()
    string(CONFIGURE [=[
@_condition@
#define USER_AHRS_DEVICE @_ahrs_backend@
#define USER_REMOTER_DR16 @_remote_dr16@
#define USER_REMOTER_VT03 @_remote_vt03@
#define USER_REMOTER_PS2 @_remote_ps2@
inline constexpr char ahrs_device[] = "@_ahrs_device@";
inline constexpr bool bmi088_heater_enabled = @_heater_enabled@;
inline constexpr imu::bmi088_thermal bmi088_thermal{@_thermal_target@, @_thermal_boost@,
    @_thermal_reboost@, @_thermal_approach_start@, @_thermal_approach_end@, @_thermal_hold_max@,
    @_thermal_kp@, @_thermal_ki@, @_thermal_kd@, @_thermal_max_out@};
inline constexpr bsp::can::config dmimu_can{bsp::can::handle::@_dm_bus@, bsp::can::id_type::standard};
inline constexpr auto rfid_uart = @_rfid_uart@;
inline constexpr buzzer::config rfid_buzzer{@_buzzer_timer@,
    bsp::pwm::channel::ch@_buzzer_channel@, @_rfid_buzzer_period_us@,
    @_rfid_buzzer_on_ms@, @_rfid_buzzer_gap_ms@};
inline constexpr auto referee_uart = @_referee_uart@;
inline constexpr bool remoter_dr16_enabled = @_enable_dr16@;
inline constexpr bool remoter_vt03_enabled = @_enable_vt03@;
inline constexpr bool remoter_ps2_enabled = @_enable_ps2@;
inline constexpr auto ui_uart = @_ui_uart@;
inline constexpr auto dr16_uart = @_dr16_uart@;
inline constexpr auto vt03_uart = @_vt03_uart@;
inline constexpr bsp::gpio::config ps2_cs{(@_ps2_cs@).instance,
    (@_ps2_cs@).pin, bsp::active_level::low};
inline constexpr auto ps2_spi = @_ps2_spi@;
]=] _branch @ONLY)
    string(APPEND _boards "${_branch}")
endforeach()
