/*----------------------------------------------------------------------------*/
/* Linker Settings                                                            */

/*----------------------------------------------------------------------------*/
/* Section Configuration                                                      */
SECTIONS
{
	.gtrackLibSection {
		_gtrackLibStart = .;
		libgtrack*.aer5f (.text)
		libgtrack*.aer5f (.rodata)
		_gtrackLibEnd = .;
	} > MSS_L2

	
	.MCPILogBuffer : {} > DSS_L3
}
/*----------------------------------------------------------------------------*/

