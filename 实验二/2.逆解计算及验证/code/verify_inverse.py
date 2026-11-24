from pathlib import Path
import sys

SECTION = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SECTION.parent / "1.正解计算及验证" / "code"))
from verification import main


if __name__ == "__main__":
    main(inverse_mode=True, output=SECTION / "results")
