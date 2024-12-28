    function targMap = targDataMap(),

    ;%***********************
    ;% Create Parameter Map *
    ;%***********************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 4;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc paramMap
        ;%
        paramMap.nSections           = nTotSects;
        paramMap.sectIdxOffset       = sectIdxOffset;
            paramMap.sections(nTotSects) = dumSection; %prealloc
        paramMap.nTotData            = -1;

        ;%
        ;% Auto data (simulink5ms_plotAndGains_P)
        ;%
            section.nData     = 4;
            section.data(4)  = dumData; %prealloc

                    ;% simulink5ms_plotAndGains_P.PacketInput1_MaxMissedTicks
                    section.data(1).logicalSrcIdx = 2;
                    section.data(1).dtTransOffset = 0;

                    ;% simulink5ms_plotAndGains_P.PacketOutput_MaxMissedTicks
                    section.data(2).logicalSrcIdx = 3;
                    section.data(2).dtTransOffset = 1;

                    ;% simulink5ms_plotAndGains_P.PacketInput1_YieldWhenWaiting
                    section.data(3).logicalSrcIdx = 4;
                    section.data(3).dtTransOffset = 2;

                    ;% simulink5ms_plotAndGains_P.PacketOutput_YieldWhenWaiting
                    section.data(4).logicalSrcIdx = 5;
                    section.data(4).dtTransOffset = 3;

            nTotData = nTotData + section.nData;
            paramMap.sections(1) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% simulink5ms_plotAndGains_P.PacketInput1_PacketID
                    section.data(1).logicalSrcIdx = 6;
                    section.data(1).dtTransOffset = 0;

                    ;% simulink5ms_plotAndGains_P.PacketOutput_PacketID
                    section.data(2).logicalSrcIdx = 7;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            paramMap.sections(2) = section;
            clear section

            section.nData     = 48;
            section.data(48)  = dumData; %prealloc

                    ;% simulink5ms_plotAndGains_P.RateTransition_InitialCondition
                    section.data(1).logicalSrcIdx = 8;
                    section.data(1).dtTransOffset = 0;

                    ;% simulink5ms_plotAndGains_P.DiscreteTransferFcn2_NumCoef
                    section.data(2).logicalSrcIdx = 9;
                    section.data(2).dtTransOffset = 1;

                    ;% simulink5ms_plotAndGains_P.DiscreteTransferFcn2_DenCoef
                    section.data(3).logicalSrcIdx = 10;
                    section.data(3).dtTransOffset = 2;

                    ;% simulink5ms_plotAndGains_P.DiscreteTransferFcn2_InitialStates
                    section.data(4).logicalSrcIdx = 11;
                    section.data(4).dtTransOffset = 4;

                    ;% simulink5ms_plotAndGains_P.Gain13_Gain
                    section.data(5).logicalSrcIdx = 12;
                    section.data(5).dtTransOffset = 5;

                    ;% simulink5ms_plotAndGains_P.Gain14_Gain
                    section.data(6).logicalSrcIdx = 13;
                    section.data(6).dtTransOffset = 6;

                    ;% simulink5ms_plotAndGains_P.DiscreteTransferFcn3_NumCoef
                    section.data(7).logicalSrcIdx = 14;
                    section.data(7).dtTransOffset = 7;

                    ;% simulink5ms_plotAndGains_P.DiscreteTransferFcn3_DenCoef
                    section.data(8).logicalSrcIdx = 15;
                    section.data(8).dtTransOffset = 9;

                    ;% simulink5ms_plotAndGains_P.DiscreteTransferFcn3_InitialStates
                    section.data(9).logicalSrcIdx = 16;
                    section.data(9).dtTransOffset = 11;

                    ;% simulink5ms_plotAndGains_P.Gain12_Gain
                    section.data(10).logicalSrcIdx = 17;
                    section.data(10).dtTransOffset = 12;

                    ;% simulink5ms_plotAndGains_P.DiscreteTransferFcn_NumCoef
                    section.data(11).logicalSrcIdx = 18;
                    section.data(11).dtTransOffset = 13;

                    ;% simulink5ms_plotAndGains_P.DiscreteTransferFcn_DenCoef
                    section.data(12).logicalSrcIdx = 19;
                    section.data(12).dtTransOffset = 15;

                    ;% simulink5ms_plotAndGains_P.DiscreteTransferFcn_InitialStates
                    section.data(13).logicalSrcIdx = 20;
                    section.data(13).dtTransOffset = 17;

                    ;% simulink5ms_plotAndGains_P.Gain_Gain
                    section.data(14).logicalSrcIdx = 21;
                    section.data(14).dtTransOffset = 18;

                    ;% simulink5ms_plotAndGains_P.Saturation_UpperSat
                    section.data(15).logicalSrcIdx = 22;
                    section.data(15).dtTransOffset = 19;

                    ;% simulink5ms_plotAndGains_P.Saturation_LowerSat
                    section.data(16).logicalSrcIdx = 23;
                    section.data(16).dtTransOffset = 20;

                    ;% simulink5ms_plotAndGains_P.DiscreteTransferFcn1_NumCoef
                    section.data(17).logicalSrcIdx = 24;
                    section.data(17).dtTransOffset = 21;

                    ;% simulink5ms_plotAndGains_P.DiscreteTransferFcn1_DenCoef
                    section.data(18).logicalSrcIdx = 25;
                    section.data(18).dtTransOffset = 22;

                    ;% simulink5ms_plotAndGains_P.DiscreteTransferFcn1_InitialStates
                    section.data(19).logicalSrcIdx = 26;
                    section.data(19).dtTransOffset = 24;

                    ;% simulink5ms_plotAndGains_P.Gain2_Gain
                    section.data(20).logicalSrcIdx = 27;
                    section.data(20).dtTransOffset = 25;

                    ;% simulink5ms_plotAndGains_P.Reference_Gain
                    section.data(21).logicalSrcIdx = 28;
                    section.data(21).dtTransOffset = 26;

                    ;% simulink5ms_plotAndGains_P.Gain3_Gain
                    section.data(22).logicalSrcIdx = 29;
                    section.data(22).dtTransOffset = 27;

                    ;% simulink5ms_plotAndGains_P.u_Gain
                    section.data(23).logicalSrcIdx = 30;
                    section.data(23).dtTransOffset = 28;

                    ;% simulink5ms_plotAndGains_P.Gain4_Gain
                    section.data(24).logicalSrcIdx = 31;
                    section.data(24).dtTransOffset = 29;

                    ;% simulink5ms_plotAndGains_P.adc_val_Gain
                    section.data(25).logicalSrcIdx = 32;
                    section.data(25).dtTransOffset = 30;

                    ;% simulink5ms_plotAndGains_P.Gain1_Gain
                    section.data(26).logicalSrcIdx = 33;
                    section.data(26).dtTransOffset = 31;

                    ;% simulink5ms_plotAndGains_P.Velocity_Gain
                    section.data(27).logicalSrcIdx = 34;
                    section.data(27).dtTransOffset = 32;

                    ;% simulink5ms_plotAndGains_P.Value_16bit2_Value
                    section.data(28).logicalSrcIdx = 35;
                    section.data(28).dtTransOffset = 33;

                    ;% simulink5ms_plotAndGains_P.Gain6_Gain
                    section.data(29).logicalSrcIdx = 36;
                    section.data(29).dtTransOffset = 34;

                    ;% simulink5ms_plotAndGains_P.Gain1_Gain_g
                    section.data(30).logicalSrcIdx = 37;
                    section.data(30).dtTransOffset = 35;

                    ;% simulink5ms_plotAndGains_P.Value_16bit3_Value
                    section.data(31).logicalSrcIdx = 38;
                    section.data(31).dtTransOffset = 36;

                    ;% simulink5ms_plotAndGains_P.Gain7_Gain
                    section.data(32).logicalSrcIdx = 39;
                    section.data(32).dtTransOffset = 37;

                    ;% simulink5ms_plotAndGains_P.Gain2_Gain_h
                    section.data(33).logicalSrcIdx = 40;
                    section.data(33).dtTransOffset = 38;

                    ;% simulink5ms_plotAndGains_P.Value_16bit1_Value
                    section.data(34).logicalSrcIdx = 41;
                    section.data(34).dtTransOffset = 39;

                    ;% simulink5ms_plotAndGains_P.Gain5_Gain
                    section.data(35).logicalSrcIdx = 42;
                    section.data(35).dtTransOffset = 40;

                    ;% simulink5ms_plotAndGains_P.Gain3_Gain_d
                    section.data(36).logicalSrcIdx = 43;
                    section.data(36).dtTransOffset = 41;

                    ;% simulink5ms_plotAndGains_P.Value_16bit4_Value
                    section.data(37).logicalSrcIdx = 44;
                    section.data(37).dtTransOffset = 42;

                    ;% simulink5ms_plotAndGains_P.Gain8_Gain
                    section.data(38).logicalSrcIdx = 45;
                    section.data(38).dtTransOffset = 43;

                    ;% simulink5ms_plotAndGains_P.Gain4_Gain_h
                    section.data(39).logicalSrcIdx = 46;
                    section.data(39).dtTransOffset = 44;

                    ;% simulink5ms_plotAndGains_P.Value_16bit5_Value
                    section.data(40).logicalSrcIdx = 47;
                    section.data(40).dtTransOffset = 45;

                    ;% simulink5ms_plotAndGains_P.Gain9_Gain
                    section.data(41).logicalSrcIdx = 48;
                    section.data(41).dtTransOffset = 46;

                    ;% simulink5ms_plotAndGains_P.Gain5_Gain_d
                    section.data(42).logicalSrcIdx = 49;
                    section.data(42).dtTransOffset = 47;

                    ;% simulink5ms_plotAndGains_P.Value_16bit6_Value
                    section.data(43).logicalSrcIdx = 50;
                    section.data(43).dtTransOffset = 48;

                    ;% simulink5ms_plotAndGains_P.Gain10_Gain
                    section.data(44).logicalSrcIdx = 51;
                    section.data(44).dtTransOffset = 49;

                    ;% simulink5ms_plotAndGains_P.Gain6_Gain_j
                    section.data(45).logicalSrcIdx = 52;
                    section.data(45).dtTransOffset = 50;

                    ;% simulink5ms_plotAndGains_P.Value_16bit7_Value
                    section.data(46).logicalSrcIdx = 53;
                    section.data(46).dtTransOffset = 51;

                    ;% simulink5ms_plotAndGains_P.Gain11_Gain
                    section.data(47).logicalSrcIdx = 54;
                    section.data(47).dtTransOffset = 52;

                    ;% simulink5ms_plotAndGains_P.Gain7_Gain_e
                    section.data(48).logicalSrcIdx = 55;
                    section.data(48).dtTransOffset = 53;

            nTotData = nTotData + section.nData;
            paramMap.sections(3) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% simulink5ms_plotAndGains_P.ConstantMustbeThisValue0x7fff_Value
                    section.data(1).logicalSrcIdx = 56;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            paramMap.sections(4) = section;
            clear section


            ;%
            ;% Non-auto Data (parameter)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        paramMap.nTotData = nTotData;



    ;%**************************
    ;% Create Block Output Map *
    ;%**************************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 3;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc sigMap
        ;%
        sigMap.nSections           = nTotSects;
        sigMap.sectIdxOffset       = sectIdxOffset;
            sigMap.sections(nTotSects) = dumSection; %prealloc
        sigMap.nTotData            = -1;

        ;%
        ;% Auto data (simulink5ms_plotAndGains_B)
        ;%
            section.nData     = 8;
            section.data(8)  = dumData; %prealloc

                    ;% simulink5ms_plotAndGains_B.RateTransition
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% simulink5ms_plotAndGains_B.Saturation
                    section.data(2).logicalSrcIdx = 3;
                    section.data(2).dtTransOffset = 1;

                    ;% simulink5ms_plotAndGains_B.DiscreteTransferFcn1
                    section.data(3).logicalSrcIdx = 4;
                    section.data(3).dtTransOffset = 2;

                    ;% simulink5ms_plotAndGains_B.Reference
                    section.data(4).logicalSrcIdx = 5;
                    section.data(4).dtTransOffset = 3;

                    ;% simulink5ms_plotAndGains_B.u
                    section.data(5).logicalSrcIdx = 6;
                    section.data(5).dtTransOffset = 4;

                    ;% simulink5ms_plotAndGains_B.adc_val
                    section.data(6).logicalSrcIdx = 7;
                    section.data(6).dtTransOffset = 5;

                    ;% simulink5ms_plotAndGains_B.Velocity
                    section.data(7).logicalSrcIdx = 8;
                    section.data(7).dtTransOffset = 6;

                    ;% simulink5ms_plotAndGains_B.RateTransition1
                    section.data(8).logicalSrcIdx = 9;
                    section.data(8).dtTransOffset = 7;

            nTotData = nTotData + section.nData;
            sigMap.sections(1) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% simulink5ms_plotAndGains_B.PacketInput1_o1
                    section.data(1).logicalSrcIdx = 10;
                    section.data(1).dtTransOffset = 0;

                    ;% simulink5ms_plotAndGains_B.PacketInput1_o3
                    section.data(2).logicalSrcIdx = 11;
                    section.data(2).dtTransOffset = 2;

            nTotData = nTotData + section.nData;
            sigMap.sections(2) = section;
            clear section

            section.nData     = 8;
            section.data(8)  = dumData; %prealloc

                    ;% simulink5ms_plotAndGains_B.PacketInput1_o2
                    section.data(1).logicalSrcIdx = 12;
                    section.data(1).dtTransOffset = 0;

                    ;% simulink5ms_plotAndGains_B.Gain1
                    section.data(2).logicalSrcIdx = 13;
                    section.data(2).dtTransOffset = 2;

                    ;% simulink5ms_plotAndGains_B.Gain2
                    section.data(3).logicalSrcIdx = 14;
                    section.data(3).dtTransOffset = 3;

                    ;% simulink5ms_plotAndGains_B.Gain3
                    section.data(4).logicalSrcIdx = 15;
                    section.data(4).dtTransOffset = 4;

                    ;% simulink5ms_plotAndGains_B.Gain4
                    section.data(5).logicalSrcIdx = 16;
                    section.data(5).dtTransOffset = 5;

                    ;% simulink5ms_plotAndGains_B.Gain5
                    section.data(6).logicalSrcIdx = 17;
                    section.data(6).dtTransOffset = 6;

                    ;% simulink5ms_plotAndGains_B.Gain6
                    section.data(7).logicalSrcIdx = 18;
                    section.data(7).dtTransOffset = 7;

                    ;% simulink5ms_plotAndGains_B.Gain7
                    section.data(8).logicalSrcIdx = 19;
                    section.data(8).dtTransOffset = 8;

            nTotData = nTotData + section.nData;
            sigMap.sections(3) = section;
            clear section


            ;%
            ;% Non-auto Data (signal)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        sigMap.nTotData = nTotData;



    ;%*******************
    ;% Create DWork Map *
    ;%*******************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 2;
        sectIdxOffset = 3;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc dworkMap
        ;%
        dworkMap.nSections           = nTotSects;
        dworkMap.sectIdxOffset       = sectIdxOffset;
            dworkMap.sections(nTotSects) = dumSection; %prealloc
        dworkMap.nTotData            = -1;

        ;%
        ;% Auto data (simulink5ms_plotAndGains_DW)
        ;%
            section.nData     = 8;
            section.data(8)  = dumData; %prealloc

                    ;% simulink5ms_plotAndGains_DW.DiscreteTransferFcn2_states
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% simulink5ms_plotAndGains_DW.DiscreteTransferFcn3_states
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

                    ;% simulink5ms_plotAndGains_DW.DiscreteTransferFcn_states
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 2;

                    ;% simulink5ms_plotAndGains_DW.DiscreteTransferFcn1_states
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 3;

                    ;% simulink5ms_plotAndGains_DW.RateTransition_Buffer0
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 4;

                    ;% simulink5ms_plotAndGains_DW.DiscreteTransferFcn3_tmp
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 5;

                    ;% simulink5ms_plotAndGains_DW.DiscreteTransferFcn_tmp
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 6;

                    ;% simulink5ms_plotAndGains_DW.RateTransition1_Buffer
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 7;

            nTotData = nTotData + section.nData;
            dworkMap.sections(1) = section;
            clear section

            section.nData     = 8;
            section.data(8)  = dumData; %prealloc

                    ;% simulink5ms_plotAndGains_DW.PacketInput1_PWORK
                    section.data(1).logicalSrcIdx = 8;
                    section.data(1).dtTransOffset = 0;

                    ;% simulink5ms_plotAndGains_DW.Scope_PWORK.LoggedData
                    section.data(2).logicalSrcIdx = 9;
                    section.data(2).dtTransOffset = 1;

                    ;% simulink5ms_plotAndGains_DW.Scope1_PWORK.LoggedData
                    section.data(3).logicalSrcIdx = 10;
                    section.data(3).dtTransOffset = 3;

                    ;% simulink5ms_plotAndGains_DW.TAQSigLogging_InsertedFor_Reference_at_outport_0_PWORK.AQHandles
                    section.data(4).logicalSrcIdx = 11;
                    section.data(4).dtTransOffset = 5;

                    ;% simulink5ms_plotAndGains_DW.TAQSigLogging_InsertedFor_Velocity_at_outport_0_PWORK.AQHandles
                    section.data(5).logicalSrcIdx = 12;
                    section.data(5).dtTransOffset = 6;

                    ;% simulink5ms_plotAndGains_DW.TAQSigLogging_InsertedFor_adc_val_at_outport_0_PWORK.AQHandles
                    section.data(6).logicalSrcIdx = 13;
                    section.data(6).dtTransOffset = 7;

                    ;% simulink5ms_plotAndGains_DW.TAQSigLogging_InsertedFor_u_at_outport_0_PWORK.AQHandles
                    section.data(7).logicalSrcIdx = 14;
                    section.data(7).dtTransOffset = 8;

                    ;% simulink5ms_plotAndGains_DW.PacketOutput_PWORK
                    section.data(8).logicalSrcIdx = 15;
                    section.data(8).dtTransOffset = 9;

            nTotData = nTotData + section.nData;
            dworkMap.sections(2) = section;
            clear section


            ;%
            ;% Non-auto Data (dwork)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        dworkMap.nTotData = nTotData;



    ;%
    ;% Add individual maps to base struct.
    ;%

    targMap.paramMap  = paramMap;
    targMap.signalMap = sigMap;
    targMap.dworkMap  = dworkMap;

    ;%
    ;% Add checksums to base struct.
    ;%


    targMap.checksum0 = 3886324679;
    targMap.checksum1 = 3302392269;
    targMap.checksum2 = 780184084;
    targMap.checksum3 = 3018656040;

