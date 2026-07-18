import IPython.display as _ipd

_STAGES = [
    ('load-overlay',  '⚙️',    'Load Overlay'),
    ('load-model',    '\U0001f4e6',       'Load Model'),
    ('setup',         '\U0001f6e0️', 'Setup Functions'),
    ('static-image',  '\U0001f5bc️', 'Static Image'),
    ('webcam-frame',  '\U0001f4f7',       'Webcam Frame'),
    ('live-detect',   '\U0001f3a5',       'Live Detection'),
]


def _render_pipeline(active):
    items = []
    for sid, icon, label in _STAGES:
        is_active = (sid == active)
        if is_active:
            bg    = 'linear-gradient(90deg,#ff6b35,#7b2d8b)'
            color = 'white'
            fw    = 'bold'
            extra = ('box-shadow:0 0 14px rgba(255,107,53,0.8);'
                     'outline:2px solid #ff6b35;outline-offset:2px;'
                     'transform:scale(1.07);')
        else:
            bg    = '#21262d'
            color = '#8b949e'
            fw    = 'normal'
            extra = ''
        items.append(
            '<div style="flex:1;min-width:88px;text-align:center;padding:10px 4px;'
            'background:' + bg + ';border-radius:10px;color:' + color + ';'
            'font-size:0.8em;font-weight:' + fw + ';' + extra + 'transition:all 0.3s;">'
            + icon + '<br><span style="font-size:0.82em;">' + label + '</span></div>'
        )
    arrow = ('<div style="color:#30363d;align-self:center;font-size:1.2em;'
             'padding:0 2px;flex-shrink:0;">&#8250;</div>')
    interleaved = []
    for i, p in enumerate(items):
        interleaved.append(p)
        if i < len(items) - 1:
            interleaved.append(arrow)
    return (
        '<div style="background:#0d1117;border:1px solid #30363d;border-radius:12px;'
        'padding:12px 16px;margin:8px 0;font-family:sans-serif;">'
        '<div style="color:#8b949e;font-size:0.7em;text-transform:uppercase;'
        'letter-spacing:1px;margin-bottom:10px;">&#128205; Pipeline Progress</div>'
        '<div style="display:flex;gap:4px;align-items:stretch;">'
        + ''.join(interleaved) +
        '</div></div>'
    )


_pipeline_handle = _ipd.display(
    _ipd.HTML(_render_pipeline(None)), display_id='pipeline-tracker'
)


def update_pipeline(stage_id):
    """Highlight *stage_id* in the pipeline progress bar displayed above."""
    _pipeline_handle.update(_ipd.HTML(_render_pipeline(stage_id)))
