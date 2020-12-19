default: dirs OpenBLAS SuiteSparse stb fast
.PHONY: dirs OpenBLAS SuiteSparse stb fast

dirs:
	@mkdir -p bin
	@mkdir -p include
	@mkdir -p lib
	@mkdir -p share

OpenBLAS:
	@echo "[Installing OpenBLAS]"
	@bash openblas.bash > openblas.log 2>&1

SuiteSparse:
	@echo "[Installing SuiteSparse]"
	@bash suitesparse.bash > suitesparse.log 2>&1

stb:
	@echo "[Installing STB]"
	@bash stb.bash > stb.log 2>&1

fast:
	@echo "[Installing FAST]"
	@bash fast.bash > stb.log 2>&1

clean:
	@rm -rf OpenBLAS
	@rm -rf SuiteSparse
	@rm -rf stb
