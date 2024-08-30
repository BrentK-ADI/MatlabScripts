%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright © 2023 by Analog Devices, Inc.  All rights reserved.
%
% This software is proprietary to Analog Devices, Inc. and its licensors.
%
% This software is provided on an “as is” basis without any representations,
% warranties, guarantees or liability of any kind.
%
% Use of the software is subject to the terms and conditions of the
% Clear BSD License ( https://spdx.org/licenses/BSD-3-Clause-Clear.html ).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef ltc2688
    %ltc2688 - Basic class for interfacing with the LTC2688/2686 parts. The
    %          class assumes the linduinoSPI or similar controller being used

    properties (Constant)
    %Device constants. Register commands and bit values

        CMD_WRITE_DAC_CODE_MASK        = 0x00
        CMD_WRITE_CH_SETTINGS_MASK     = 0x10
        CMD_WRITE_OFFSET_ADJ_MASK      = 0x20
        CMD_WRITE_GAIN_ADJ_MASK        = 0x30
        CMD_WRITE_CODE_UPDATE_MASK     = 0x40
        CMD_WRITE_CODE_UPDATE_ALL_MASK = 0x50
        CMD_UPDATE_CHANNEL_MASK        = 0x60
        CMD_WRITE_CONFIG               = 0x70
        CMD_WRITE_POWER_DOWN           = 0x71
        CMD_WRITE_AB_SELECT            = 0x72
        CMD_WRITE_SW_TOGGLE            = 0x73
        CMD_WRITE_DITHER               = 0x74
        CMD_WRITE_MUX_CTRL             = 0x75
        CMD_WRITE_FAULT                = 0x76
        CMD_WRITE_CODE_ALL             = 0x78
        CMD_WRITE_CODE_ALL_UPDATE      = 0x79
        CMD_WRITE_SETTINGS_ALL         = 0x7A
        CMD_WRITE_SETTINGS_UPDATE_ALL  = 0x7B
        CMD_UPDATE_ALL                 = 0x7C

        %Values for the Span field of the settins register
        SPAN_0_5V    = 0x00
        SPAN_0_10V   = 0x01
        SPAN_N5_5V   = 0x02
        SPAN_N10_10V = 0x03
        SPAN_N15_15V = 0x04
        SPAN_5PERCENT_OVER_BIT = 0x08 %Bit to include 5% over range

        %Specify the bit depth of this chip
        DEPTH_12_BIT = 0
        DEPTH_16_BIT = 1

        %Specify the device type
        DEVICE_LTC2686 = 0
        DEVICE_LTC2688 = 1
    end

    properties (Access = private)
        controller %SPI controller
        depth      %Bit depth
        device     %Device Type
    end

    methods
        function self = ltc2688(controller, device, depth)
        %ltc2688 - Class constructor. Just assign the controller
        %
        % Inputs
        %   controller - linduinoSPI instance, or other controller implemeting
        %                the writeRead method
        %   device     - Device type, Use DEVICE_ constant
        %   depth      - Bit depth of the chip. Use DEPTH_ constant

            self.controller = controller;
            self.device = device;
            self.depth = depth;
        end

        function setSpan(self, ch, span)
        %setSpan - Method to set the span value for the provided channel
        %
        % Inputs
        %    ch - Channel to set. 0 based
        %    span - Span to set. use the SPAN_ constants

            %For the LTC2686, the address is shift left by 1
            if(self.device == self.DEVICE_LTC2686)
                ch = bitshift(ch, 1);
            end
            self.writeReg(bitor(self.CMD_WRITE_CH_SETTINGS_MASK, ch), span);
        end

        function setChannelCode(self, ch, code)
        %setChannelCode - Method to set the span value for the provided channel
        %                 NOTE: Update needs to be called on all or the channel
        %                 for the value to take affect.
        %
        % Inputs
        %    ch - Channel to set. 0 based
        %    span - Span to set. use the SPAN_ constants

            %For the LTC2686, the address is shift left by 1
            if(self.device == self.DEVICE_LTC2686)
                ch = bitshift(ch, 1);
            end

            %12-bit data is left justified, so shift left by 4
            if( self.depth == self.DEPTH_12_BIT )
                code = bitshift(code, 4);
            end

            self.writeReg(bitor(self.CMD_WRITE_DAC_CODE_MASK, ch), code);
        end

        function updateAll(self)
        % updateAll - Sends an update request to all channels to update the DAC
        %             output to the latest register value
            self.writeReg(self.CMD_UPDATE_ALL, 0x00);
        end

        function writeReg(self, reg, value)
        % writeReg - Generic method for writing a chip register
        %
        % Inputs
        %    reg - Register to write
        %    value - Value to write, 16-bits

            %Get the individual bytes from the value
            regVal = typecast(uint16(value),'uint8');

            %Build the SPI frame and send
            data = [reg, regVal(2), regVal(1)];
            self.controller.writeRead(data);
        end
    end
end

