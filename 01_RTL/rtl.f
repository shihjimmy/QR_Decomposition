+v2k 
-debug_access+all 
+notimingcheck 
-P /usr/cad/synopsys/verdi/cur/share/PLI/VCS/LINUX64/novas.tab
/usr/cad/synopsys/verdi/cur/share/PLI/VCS/LINUX64/pli.a
-sverilog 
-assert svaext
+lint=TFIPC-L
+fsdb+parameter=on

-y /usr/cad/synopsys/synthesis/cur/dw/sim_ver +libext+.v
+incdir+/usr/cad/synopsys/synthesis/cur/dw/sim_ver/+

// Change different 2ackets
//+define+P6

// Add your RTL & SRAM files
QR_Engine.v
../SRAM/sram_256x8/sram_256x8.v
// tb
testfixture.v