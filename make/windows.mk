############################
# Configure shell commands
############################

RM := del
MKDIR := powershell -noprofile -command New-Item -Force -itemtype "directory"


.PHONY: copy_src_files
copy_src_files:
	@echo Copying source files...
	@python tools/copy_files.py $(CURDIR) $(BUILD_DIR)