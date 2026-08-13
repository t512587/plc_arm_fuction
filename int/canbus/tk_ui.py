from __future__ import annotations

from tkinter import font as tkfont
from tkinter import ttk


CJK_FONT_CANDIDATES = (
    "Microsoft JhengHei UI",
    "Microsoft JhengHei",
    "PingFang TC",
    "Heiti TC",
    "Noto Sans CJK TC",
    "Noto Sans CJK SC",
    "Source Han Sans TW",
    "Source Han Sans TC",
    "WenQuanYi Zen Hei",
    "Droid Sans Fallback",
)


def configure_tk_ui(root) -> tuple[tkfont.Font, tkfont.Font, tkfont.Font]:
    """Apply a readable Tk/ttk font setup, including CJK fallback."""

    available_families = set(tkfont.families(root))
    selected_family = next(
        (family for family in CJK_FONT_CANDIDATES if family in available_families),
        tkfont.nametofont("TkDefaultFont").actual("family"),
    )

    for font_name in (
        "TkDefaultFont",
        "TkTextFont",
        "TkMenuFont",
        "TkHeadingFont",
        "TkCaptionFont",
        "TkSmallCaptionFont",
        "TkIconFont",
        "TkTooltipFont",
    ):
        try:
            tkfont.nametofont(font_name).configure(family=selected_family, size=11)
        except Exception:  # noqa: BLE001
            continue

    default_font = tkfont.Font(family=selected_family, size=11)
    heading_font = tkfont.Font(family=selected_family, size=11, weight="bold")
    mono_font = tkfont.Font(family=selected_family, size=10)

    root.option_add("*Font", default_font)

    style = ttk.Style(root)
    style.configure(".", font=default_font)
    style.configure("TButton", padding=(8, 4))
    style.configure("TCombobox", padding=(4, 2))
    style.configure("Treeview", font=default_font, rowheight=28)
    style.configure("Treeview.Heading", font=heading_font)

    return default_font, heading_font, mono_font
