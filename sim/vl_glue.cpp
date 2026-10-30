// Verilator wants this from every program; the testbenches keep no SystemC time
double sc_time_stamp() { return 0; }
