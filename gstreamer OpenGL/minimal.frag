#ifdef GL_ES
precision mediump float;
#endif
//varying vec2 v_texcoord;
uniform sampler2D tex;
uniform float time;
uniform float width;
uniform float height;

vec2 v_texcoord = gl_TexCoord[0].xy;

void main () {
    vec2 tc = vec2(v_texcoord[1], v_texcoord[0]);
  gl_FragColor = texture2D(tex, tc);
}
