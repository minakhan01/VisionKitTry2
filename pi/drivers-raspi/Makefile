PI ?= raspberrypi.local

deploy:
	git ls-files | rsync -avz --exclude=".*" --files-from - . pi@$(PI):~/drivers-raspi
