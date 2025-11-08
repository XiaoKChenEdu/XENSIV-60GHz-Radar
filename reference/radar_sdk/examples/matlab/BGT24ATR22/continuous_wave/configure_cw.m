%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function:      RunMeasurement_BGT24ATR22
% Author:        Adeel Jalil
% Description:   Runs a (radar mode) measurement with the gimbal
% Date:          2024/04/18
% Last Changes:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Transmit_antenna = 1;
Transmit_antenna = 2;


if(exist('Dev','var'))
    clear Dev;
end
%##########################################################################
% Check for library dependancies
%##########################################################################
try
    MimoseDevice.check_path();
catch
    ME = MException(['RadarPath:error'], 'ATR22 MexWrapper not found, add path to MimoseSDKMEXWrapper');
    throw(ME);
end
try
    disp(['Radar SDK Version: ' MimoseDevice.get_version_full()]);
catch
    ME = MException(['RadarDevice:error'], 'SDK library not found, add path containing compiled ATR22 Mex file and RDK libraries');
    throw(ME);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% configure Mimose Device
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Dev = MimoseDevice();
% Modify default configuration
Dev.set_config('BGT24ATR22_continuous_wave.json');

Dev.set_register('SEQ_MAIN_CONF', 0x0001);
%% set Radar in CW mode
Dev.set_register('VCO_TEST0', 0x03DF);
Dev.set_register('SEQ_EN_OW', 0x0063);
Dev.set_register('TX_CONF', 0x007B)

if(Transmit_antenna == 1)
    % Enable TX1
    Dev.set_register('TX1_PC0_CONF', 0x007F);
    Dev.set_register('TX1_TEST', 0x0FFF);
    Dev.set_register('TX2_TEST', 0x07FC);
else
    % Enable TX2
    Dev.set_register('TX2_PC0_CONF', 0x007F);
    Dev.set_register('TX1_TEST', 0x07FF);
    Dev.set_register('TX2_TEST', 0x0FFC);
end

Dev.registers.view_registers('separator',';','to_file',1,'filename','atr22_regs.csv');

%##########################################################################
% STEP 7: Clear the RadarDevice object. It also automatically disconnects
% from the device.
%##########################################################################
clear Dev
        
        

        
 