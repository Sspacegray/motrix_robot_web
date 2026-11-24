// Brand provenance regression. Local files only, no network or browser.
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
const hash = bytes => createHash('sha256').update(bytes).digest('hex');
test('header and favicon use authentic copied Motphys SVG assets', async () => {
  const html=await readFile(new URL('../frontend/index.html',import.meta.url),'utf8');
  const css=await readFile(new URL('../frontend/css/style.css',import.meta.url),'utf8');
  assert.match(html,/class="motphys-brand-asset" src="\/assets\/motphys-logo\.svg(?:\?[^" ]+)?"/);
  assert.match(html,/href="\/assets\/motphys-icon\.svg(?:\?[^" ]+)?"/);
  const brand=html.split('class="brand-logo"')[1].split('</a>')[0];
  assert.doesNotMatch(brand,/<svg|<polygon/);
  assert.match(css,/\.motphys-brand-asset\s*\{[^}]*height:20px;[^}]*width:auto;[^}]*filter:brightness\(0\)/);
});
test('vendored brand bytes stay identical to reviewed original assets', async () => {
  const expected = {
    'motphys-logo.svg':'4beb55868b26bf994b4e4a298189d992ff455a5e352deff65c5b766076784fdc',
    'motphys-icon.svg':'3614f59afbc31eff6d39a0e4f127736f15bb8e51c3dfdda3f95f266c6aa55545'
  };
  for (const [name,digest] of Object.entries(expected)) {
    const bytes=await readFile(new URL('../frontend/assets/'+name,import.meta.url));
    assert.equal(hash(bytes),digest,name);
    assert.doesNotMatch(bytes.toString(),/<script|<foreignObject|href=|<!ENTITY/i);
  }
});
