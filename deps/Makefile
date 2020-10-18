default: OpenBLAS SuiteSparse
.PHONY: OpenBLAS SuiteSparse

OpenBLAS:
	@echo "[Installing OpenBLAS]"
	@bash openblas.bash

SuiteSparse:
	@echo "[Installing SuiteSparse]"
	@bash SuiteSparse.bash

clean:
	@rm -rf OpenBLAS
	@rm -rf SuiteSparse
