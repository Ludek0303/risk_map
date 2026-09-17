-- suas_comms_failsafe.lua
--
-- SUAS communication-loss failsafe
--
-- Założenia:
--
-- Utrata RC (GCS/telemetria nieużywana w detekcji):
--
-- 0 - 15 s:
--     brak automatycznej zmiany trybu
--
-- 15 s:
--     RTL
--
-- 90 s:
--     Flight Termination
--     = natychmiastowe DISARM / zatrzymanie silników
--
-- Jeśli komunikacja wróci przed 90 s:
--     anulujemy termination.
--
-- Jeżeli RTL już został rozpoczęty,
-- NIE wracamy automatycznie do AUTO.
--
-- ========================================================


------------------------------------------------------------
-- USTAWIENIA
------------------------------------------------------------

local MODE_RTL = 6

-- RTL po 15 sekundach
local RTL_TIME_MS = 15000

-- Flight Termination po 90 sekundach
-- WAŻNE: jest to 90 sekund OD POCZĄTKU utraty komunikacji,
-- a nie 90 sekund po RTL.
local TERMINATION_TIME_MS = 90000


------------------------------------------------------------
-- STAN
------------------------------------------------------------

local loss_active = false
local loss_start = 0

local rtl_sent = false


------------------------------------------------------------
-- FLIGHT TERMINATION
------------------------------------------------------------

local function terminate_flight()

    gcs:send_text(
        0,
        "SUAS FS: FLIGHT TERMINATION"
    )

    --------------------------------------------------------
    -- arming:disarm() w Lua rozbraja pojazd także w locie.
    --
    -- Dla multirotora oznacza to zatrzymanie silników.
    --------------------------------------------------------

    if arming:is_armed() then

        local success = arming:disarm()

        if success then

            gcs:send_text(
                0,
                "SUAS FS: MOTORS STOPPED"
            )

        else

            gcs:send_text(
                0,
                "SUAS FS: DISARM FAILED"
            )

        end

    end
end


------------------------------------------------------------
-- GŁÓWNA PĘTLA
------------------------------------------------------------

local function update()

    local now = millis()


    --------------------------------------------------------
    -- RC
    --------------------------------------------------------

    local rc_ok = rc:has_valid_input()


    --------------------------------------------------------
    -- DEFINICJA COMMS LOSS
    --
    -- tylko RC; GCS/telemetria nie jest sprawdzana.
    --------------------------------------------------------

    local comms_lost = not rc_ok


    --------------------------------------------------------
    -- KOMUNIKACJA DZIAŁA
    --------------------------------------------------------

    if not comms_lost then

        ----------------------------------------------------
        -- Jeśli wcześniej był failsafe, anulujemy timer.
        ----------------------------------------------------

        if loss_active then

            gcs:send_text(
                4,
                "SUAS FS: communications restored"
            )

            loss_active = false
            rtl_sent = false
            loss_start = 0

        end

        return update, 100
    end


    --------------------------------------------------------
    -- POCZĄTEK UTRATY KOMUNIKACJI
    --------------------------------------------------------

    if not loss_active then

        loss_active = true
        loss_start = now
        rtl_sent = false

        gcs:send_text(
            4,
            "SUAS FS: COMMS LOST - timer started"
        )

        return update, 100
    end


    --------------------------------------------------------
    -- CZAS UTRATY KOMUNIKACJI
    --------------------------------------------------------

    local elapsed = now - loss_start


    --------------------------------------------------------
    -- 15 SEKUND -> RTL
    --------------------------------------------------------

    if elapsed >= RTL_TIME_MS and not rtl_sent then

        rtl_sent = true

        if vehicle:get_mode() ~= MODE_RTL then

            if vehicle:set_mode(MODE_RTL) then

                gcs:send_text(
                    2,
                    "SUAS FS: 15s COMMS LOSS -> RTL"
                )

            else

                gcs:send_text(
                    0,
                    "SUAS FS: RTL FAILED"
                )

            end

        end
    end


    --------------------------------------------------------
    -- 90 SEKUND -> FLIGHT TERMINATION
    --------------------------------------------------------

    if elapsed >= TERMINATION_TIME_MS then

        terminate_flight()

        ----------------------------------------------------
        -- Po termination nie potrzebujemy już prowadzić
        -- kolejnych akcji.
        ----------------------------------------------------

        loss_active = false

        return update, 1000
    end


    return update, 100
end


------------------------------------------------------------
-- START
------------------------------------------------------------

gcs:send_text(
    4,
    "SUAS comms failsafe loaded"
)

return update()
