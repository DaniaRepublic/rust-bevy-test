.PHONY: run all

run:
	MTL_HUD_ENABLED=1 cargo run --features bevy/dynamic_linking

run-coacd:
	python3.12 ./tools/mesh-utils/coacd/main.py

run-open3d:
	python3.12 ./tools/mesh-utils/open3d/main.py
