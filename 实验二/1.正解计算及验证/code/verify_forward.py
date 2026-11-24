from pathlib import Path

from verification import main


if __name__ == "__main__":
    main(inverse_mode=False, output=Path(__file__).resolve().parents[1] / "results")
