board = arduino:avr:uno
board_infix = $(subst :,.,$(board))
sketch = arduinoMain
flags = --fqbn
port = /dev/ttyACM0

src = $(sketch)/$(sketch).ino $(wildcard $(sketch)/*.cpp) $(wildcard $(sketch)/*.h)
targets = $(addprefix $(sketch)/$(sketch).$(board_infix),.hex .with_bootloader.hex .elf)

all : $(targets)
upload : $(targets)
	$(info Uploading sketch to $(port)...)
	arduino-cli upload -p $(port) $(flags) $(board) $(sketch)
$(targets) & : $(src)
	arduino-cli compile $(flags) $(board) $(sketch)
clean:
	rm $(targets)




