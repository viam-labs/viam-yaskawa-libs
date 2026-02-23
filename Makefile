SRCS := $(shell find . \( -path "./build" -o -path "./logging/build" \) -prune -o -name "*.c" -print -o -name "*.h" -print )

format:
	@echo "Formatting C/H files..."
	@clang-format -i $(SRCS)
format-check:
	@echo "Checking C/H file formatting..."
	@clang-format --dry-run --Werror $(SRCS) && echo "All files are properly formatted!" || (echo "Some files need formatting. Run 'make format' to fix." && exit 1)

.PHONY: format format-check
