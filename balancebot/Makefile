all:
#	@make -C xbee_serial --no-print-directory
#	@make -C test_xbee_receive --no-print-directory
	@make -C measure_moments --no-print-directory
	@make -C measure_motors --no-print-directory
	@make -C test_motors --no-print-directory
	@make -C balancebot --no-print-directory


opti:
#	@make -C xbee_serial --no-print-directory
#	@make -C test_xbee_receive --no-print-directory
	@make -C optitrack/common --no-print-directory
	@make -C optitrack --no-print-directory

clean:                                             
	@make -C balancebot -s clean
	@make -C measure_motors -s clean
	@make -C measure_moments -s clean
	@make -C test_motors -s clean
	@make -C xbee_serial -s clean
	@make -C test_xbee_receive -s clean
	@make -C optitrack -s clean
	@make -C optitrack/common -s clean
