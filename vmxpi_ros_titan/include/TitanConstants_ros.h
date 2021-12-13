#pragma once

class TitanMessageID {
    public:

        static constexpr int baseID         = 0x2000000 + 0xC0000;

        static constexpr int DISABLED_FLAG            = baseID + (64*0);
        static constexpr int ENABLED_FLAG             = baseID + (64*1);
        static constexpr int SET_MOTOR_SPEED          = baseID + (64*2);
        static constexpr int DISABLE_MOTOR            = baseID + (64*3);         
        static constexpr int GET_TITAN_INFO           = baseID + (64*4);        
        static constexpr int RETURN_TITAN_INFO        = baseID + (64*5);
        static constexpr int GET_UNIQUE_ID            = baseID + (64*6);
        static constexpr int RETURN_WORD_1            = baseID + (64*7);
        static constexpr int RETURN_WORD_2            = baseID + (64*8);
        static constexpr int RETURN_WORD_3            = baseID + (64*9);
        static constexpr int CONFIGURE_MOTOR          = baseID + (64*10);
        static constexpr int GET_MOTOR_FREQUENCY      = baseID + (64*11);
        static constexpr int RETURN_MOTOR_FREQUENCY   = baseID + (64*12);
        static constexpr int RESET_ENCODER            = baseID + (64*13);
        static constexpr int SET_CURRENT_LIMIT        = baseID + (64*14);
        static constexpr int SET_MOTOR_STOP_MODE      = baseID + (64*15);
        static constexpr int SET_TARGET_VELOCITY      = baseID + (64*16);
        static constexpr int SET_TARGET_DISTANCE      = baseID + (64*17);

        static constexpr int ENCODER_0_OUTPUT         = baseID + (64*37);
        static constexpr int ENCODER_1_OUTPUT         = baseID + (64*38);
        static constexpr int ENCODER_2_OUTPUT         = baseID + (64*39);
        static constexpr int ENCODER_3_OUTPUT         = baseID + (64*40);
        static constexpr int RPM_0_OUTPUT             = baseID + (64*41);
        static constexpr int RPM_1_OUTPUT             = baseID + (64*42);
        static constexpr int RPM_2_OUTPUT             = baseID + (64*43);
        static constexpr int RPM_3_OUTPUT             = baseID + (64*44);
        static constexpr int LIMIT_SWITCH_OUTPUT      = baseID + (64*45);
        static constexpr int CURRENT_OUTPUT           = baseID + (64*46);
        static constexpr int MCU_TEMP                 = baseID + (64*47);
        static constexpr int MAX_RANGE                = baseID + (64*1023);

};