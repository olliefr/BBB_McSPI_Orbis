# BBB_McSPI_Orbis

**Oliver Frolovs**

Experimenting with different operation regimes for McSPI controller on BeagleBone Black (AM335x CPU). Orbis rotary encoder serves as a peripheral.

The 'Bone CPU has a Technical Reference Manual, but SPI module is not well explained, in my opinion. I've been horsing around with the TI XDS110 Debug Probe, working out the correct sequence of operations and clarifying the TI StarterWare API for working with McSPI.

I also have a log file with my findings, which I must edit before adding to this repo. It contains the findings and lists the caveats I have found while experimenting.

The tested operation regimes each have a tag whose name and the commit message explains the configuration parameters.

Signed,
Oliver
