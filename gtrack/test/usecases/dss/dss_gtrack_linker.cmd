/*----------------------------------------------------------------------------*/
/* Linker Settings                                                            */
--retain="*(.intvecs)"

-stack 0x1000

/*----------------------------------------------------------------------------*/
/* Section Configuration                                                      */
SECTIONS
{
    systemHeap : {} >> DSS_L2
	.gtrackLibSection {
		_gtrackLibStart = .;
		libgtrack*.ae66 (.text)
		libgtrack*.ae66 (.const)
		_gtrackLibEnd = .;
	} > DSS_L2
	
	.MCPILogBuffer : {} > DSS_L3
}
/*----------------------------------------------------------------------------*/

