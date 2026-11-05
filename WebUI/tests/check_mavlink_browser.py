from playwright.sync_api import sync_playwright
import json
import argparse
import tempfile
from pathlib import Path

parser = argparse.ArgumentParser(description="Run against a local --demo monitor; requires Playwright.")
parser.add_argument("--url", default="http://127.0.0.1:8081")
parser.add_argument("--chromium", default=None, help="Optional existing Chromium executable")
args = parser.parse_args()
artifacts = Path(tempfile.mkdtemp(prefix="mavlink-ui-"))
with sync_playwright() as p:
    browser = p.chromium.launch(executable_path=args.chromium, headless=True, args=['--no-sandbox'])
    page = browser.new_page(viewport={'width': 1512, 'height': 1050}, device_scale_factor=1)
    errors = []
    page.on('pageerror', lambda error: (errors.append(str(error)), print('BROWSER ERROR:', str(error), flush=True)))
    page.goto(args.url, wait_until='domcontentloaded')
    page.wait_for_function("document.querySelector('#fieldsMetric').textContent === '51'")
    page.wait_for_function("document.querySelectorAll('.legend-item').length === 24")
    assert page.locator('.chart-card').count() == 4
    page.wait_for_function("[...document.querySelectorAll('.legend-value')].some(el => el.textContent !== '—' && el.textContent !== '')")
    page.screenshot(path=str(artifacts / 'desktop.png'), full_page=True)
    # Freedom to add/remove fields from an independent chart.
    page.locator('#addChart').click()
    assert page.locator('.chart-card').count() == 5
    page.locator('#fieldSearch').fill('A_ACC_R_X')
    page.get_by_role('checkbox', name='A_ACC_R_X', exact=True).check()
    active = page.locator('.chart-card.active')
    assert active.locator('.legend-item').count() == 1
    active.locator('.chart-title').fill('自定义 X 对比')
    active.locator('.chart-title').press('Tab')
    page.locator('#fieldSearch').fill('A_ACC_C_X')
    page.get_by_role('checkbox', name='A_ACC_C_X', exact=True).check()
    assert active.locator('.legend-item').count() == 2
    page.locator('#pauseButton').click()
    assert page.locator('#liveState').inner_text() == '历史查看'
    canvas = active.locator('canvas')
    canvas.hover()
    active.locator('.plot-tooltip').wait_for(state='visible')
    with page.expect_download() as download:
        page.locator('#exportCsv').click()
    download.value.save_as(str(artifacts / 'visible-signals.csv'))
    csv = open(str(artifacts / 'visible-signals.csv'), encoding='utf-8-sig').read()
    assert 'A_ACC_R_X' in csv and 'A_ACC_C_X' in csv
    page.locator('#resetView').click()
    assert page.locator('#liveState').inner_text() == '实时跟随'
    # Fixed axes, wheel zoom, and double-click reset.
    active.locator('.chart-scale').click()
    active.locator('.y-min').fill('-2'); active.locator('.y-max').fill('2')
    active.locator('.scale-apply').click()
    canvas.hover(); page.mouse.wheel(0, -200)
    page.wait_for_timeout(200)
    assert page.locator('#liveState').inner_text() == '历史查看'
    canvas.dblclick()
    assert page.locator('#liveState').inner_text() == '实时跟随'
    # Saved layout survives reload.
    page.reload(wait_until='domcontentloaded')
    page.wait_for_function("document.querySelectorAll('.chart-card').length === 5")
    assert page.locator('.chart-card').count() == 5
    assert page.locator('.chart-title').last.input_value() == '自定义 X 对比'
    page.set_viewport_size({'width': 390, 'height': 844})
    page.screenshot(path=str(artifacts / 'mobile.png'), full_page=True)
    overflow = page.evaluate('document.documentElement.scrollWidth > innerWidth')
    assert not overflow, 'mobile horizontal overflow'
    # New CEKF presets resolve named fusion/status channels without manual selection.
    page.set_viewport_size({'width': 1512, 'height': 1050})
    page.locator('#preset').select_option('cekf')
    page.wait_for_function("document.querySelectorAll('.chart-card').length === 6 && document.querySelectorAll('.legend-item').length === 18")
    page.locator('#fieldSearch').fill('')
    page.locator('[data-filter="cekf"]').click()
    page.wait_for_function("document.querySelector('#signalCount').textContent === '27'")
    page.wait_for_function("[...document.querySelectorAll('.legend-value')].every(el => el.textContent !== '—' && el.textContent !== '')")
    page.screenshot(path=str(artifacts / 'cekf-fusion.png'), full_page=True)
    page.locator('#preset').select_option('cekf-status')
    page.wait_for_function("document.querySelectorAll('.chart-card').length === 7 && document.querySelectorAll('.legend-item').length === 13")
    with page.expect_download() as download:
        page.locator('#exportCsv').click()
    download.value.save_as(str(artifacts / 'cekf-status.csv'))
    status_csv = (artifacts / 'cekf-status.csv').read_text(encoding='utf-8-sig')
    assert 'CE_W_A' in status_csv and 'CE_LINK' in status_csv
    page.screenshot(path=str(artifacts / 'cekf-status.png'), full_page=True)
    page.set_viewport_size({'width': 390, 'height': 844})
    assert not page.evaluate('document.documentElement.scrollWidth > innerWidth')
    assert not errors, errors
    print(json.dumps({'browser_errors': errors, 'charts': page.locator('.chart-card').count(), 'fields': page.locator('#fieldsMetric').inner_text(), 'csv_bytes': len(csv), 'mobile_overflow': overflow, 'artifacts': str(artifacts)}, ensure_ascii=False))
    browser.close()
