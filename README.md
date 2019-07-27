# cy8c95xx - Cypress CY8C95XX I/O expander linux driver

Linux driver support for Cypress CY8C9520 / CY8C9540 / CY8C9560 i2c I/O expanders.

Currently only tested on CY8C9540a.


The driver makes no distinction between the three components, it just declares the maximum theorical number of GPIOs (8x8 bits port = 64 pins) and consider the user knows the platform used / what to do.

Port 0 is GPIO numbers 0 to 7.
Port 1 is 8 to 15.
Port 2 is 16 to 23.
Port 3 is 24 to 31.
Port 4 is 32 to 39.
Port 5 is 40 to 47.
Port 6 is 48 to 55.
Port 7 is 56 to 63.


## Supported features:
  - GPIOs:
    - Direction setting: input / output
    - Input reading
    - Output setting
    - Bidirectional read / write
  - Line drive mode:
    - High Z
    - Pull down
    - Pull up
    - Strong push / pull
  - Interrupts
    
    
## UNSUPPORTED features:
  - PWMs
  - Line drive mode:
    - "Open drain high" (see datasheet)
    - "Open drain low" (see datasheet)
    - "Slow strong" (see datasheet)
  - EEPROM: 
    - Reading / writing
    - Config save / load
  - Watchdog
  
## DTS example file (raspberry pi zero w)

See cy8c95xx.dts file for latest version. 

  
```
/dts-v1/;
/plugin/;

/ {
	compatible = "brcm,bcm2708";
	
	fragment@0 {
		target = <&i2c1>;

		__overlay__ {
			status = "okay";
			#address-cells = <1>;
			#size-cells = <0>;
			cy8c95xx: cy8c95xx@20 {
				compatible = "cypress,cy8c95xx";
				reg = <0x20>;
        
				gpio-controller;
				#gpio-cells = <2>;
        
				interrupt-controller;
				interrupt-parent = <&gpio>;
				#interrupt-cells = <2>;
				interrupts = <21 1>;  // 1 = edge rising
			};
		};
	};
};
```
