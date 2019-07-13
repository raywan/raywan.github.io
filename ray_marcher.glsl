#define AA 2
#define MAX_MARCHING_STEPS 255
#define MIN_DIST 0.0
#define MAX_DIST 100.0
#define PI 3.14159265359
#define deg_to_rad(deg) (deg * PI/180.0)
#define EPS 0.0001


mat4 inv_rotate_x(in float deg) {
  float c = cos(deg_to_rad(-deg));
  float s = sin(deg_to_rad(-deg));

  return mat4(
    vec4(1, 0, 0, 0),
    vec4(0, c, -s, 0),
    vec4(0, s, c, 0),
    vec4(0, 0, 0, 1)
  );
}

mat4 inv_rotate_y(in float deg) {
  float c = cos(deg_to_rad(-deg));
  float s = sin(deg_to_rad(-deg));

  return mat4(
    vec4(c, 0, s, 0),
    vec4(0, 1, 0, 0),
    vec4(-s, 0, c, 0),
    vec4(0, 0, 0, 1)
  );
}

mat4 inv_rotate_z(in float deg) {
  float c = cos(deg_to_rad(-deg));
  float s = sin(deg_to_rad(-deg));

  return mat4(
    vec4(c, -s, 0, 0),
    vec4(s, c, 0, 0),
    vec4(0, 0, 1, 0),
    vec4(0, 0, 0, 1)
  );
}

mat4 inv_translate(in vec3 v) {
  return mat4(
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    -v.x, -v.y, -v.z, 1.0
  );
}


float union_sdf(float dist_a, float dist_b) {
  return min(dist_a, dist_b);
}

float smooth_union_sdf(float d1, float d2, float k) {
    float h = max( k-abs(d1-d2), 0.0 );
    return min( d1, d2 ) - h*h*0.25/k;
}

float sphere_sdf(vec3 sample_point, float radius) {
  return length(sample_point) - radius;
}

float vert_capsule_sdf(vec3 p, float h, float r) {
  p.y -= clamp( p.y, 0.0, h );
  return length( p ) - r;
}

float torus_sdf(vec3 p, vec2 t) {
  vec2 q = vec2(length(p.xz)-t.x,p.y);
  return length(q)-t.y;
}

float r_sdf(vec3 pt, vec3 pos) {
  float r_vc_1_dist = vert_capsule_sdf(
    (inv_translate(vec3(pos.x-1.0,pos.y,pos.z)) * vec4(pt,1.0)).xyz,
    3.4, 0.4);
  float r_vc_2_dist = vert_capsule_sdf(
    (inv_rotate_z(-40.0)*inv_translate(vec3(pos.x+1.2,pos.y,pos.z))*vec4(pt,1.0)).xyz,
    1.4, 0.4);
  float r_torus_dist = torus_sdf(
    (inv_rotate_x(90.0)*inv_translate(vec3(pos.x,pos.y+2.4,pos.z))*vec4(pt,1.0)).xyz,
    vec2(0.95, 0.35));
  return smooth_union_sdf(r_vc_1_dist, smooth_union_sdf(r_vc_2_dist, r_torus_dist, 0.5), 0.3);
}

float a_sdf(vec3 pt, vec3 pos) {
  float a_vc_1_dist = vert_capsule_sdf(
    (inv_translate(vec3(pos.x+1.2,pos.y,pos.z))*vec4(pt,1.0)).xyz,
    1.8, 0.4);
  float a_torus_dist = torus_sdf(
    (inv_rotate_x(90.0)*inv_translate(vec3(pos.x,pos.y+1.0,pos.z))*vec4(pt,1.0)).xyz,
    vec2(1.0, 0.35));
  return smooth_union_sdf(a_vc_1_dist, a_torus_dist, 0.2);
}

float y_sdf(vec3 pt, vec3 pos) {
  float y_vc_1_dist = vert_capsule_sdf(
    (inv_rotate_z(-30.0)*inv_translate(vec3(pos.x,pos.y,pos.z))*vec4(pt,1.0)).xyz,
    2.0, 0.4);
  float y_vc_2_dist = vert_capsule_sdf(
    (inv_rotate_z(20.0)*inv_translate(vec3(pos.x-0.3,pos.y-2.0,pos.z)) * vec4(pt,1.0)).xyz,
    4.0, 0.4);
  return smooth_union_sdf(y_vc_1_dist, y_vc_2_dist, 0.3);
}

float scene_name_sdf(vec3 pt) {
  return union_sdf(
      r_sdf(pt, vec3(0)),
      union_sdf(a_sdf(pt, vec3(3.0, 0.0,0)), y_sdf(pt, vec3(6.3,0,0)))
  );
}

float scene_sdf(in vec3 pos) {
    return scene_name_sdf(pos -vec3( -3.0, -1.0, 10.0));
    return sphere_sdf(pos-vec3( 0.0, 0.0, 2.0), 0.25);
}

vec3 estimate_normal(vec3 p) {
  return normalize(vec3(
    scene_sdf(vec3(p.x + EPS, p.y, p.z)) - scene_sdf(vec3(p.x - EPS, p.y, p.z)),
    scene_sdf(vec3(p.x, p.y + EPS, p.z)) - scene_sdf(vec3(p.x, p.y - EPS, p.z)),
    scene_sdf(vec3(p.x, p.y, p.z  + EPS)) - scene_sdf(vec3(p.x, p.y, p.z - EPS))
  ));
}

float march_ray(in vec3 ray_origin, in vec3 ray_dir) {
    float t = MIN_DIST;
    float t_max = MAX_DIST;
    float t_min = MIN_DIST;
    float tp1 = (0.0-ray_origin.y)/ray_dir.y;
    if( tp1 > 0.0 )
    {
        t_max = min( tp1, t_max );
        t = tp1;
    }

    for (int i = 0; i < MAX_MARCHING_STEPS; i++) {
      float dist = scene_sdf(ray_origin + t * ray_dir);
      if (dist < 0.0001) {
          return t;
      }
      t += dist;
      if (t >= t_max) {
          return t_max;
      }
    }
    return t;
    // return vec2(0.0);
}

float maxHei = 0.8;

// http://iquilezles.org/www/articles/rmshadows/rmshadows.htm
float calcSoftshadow( in vec3 ro, in vec3 rd, in float mint, in float tmax )
{

    // bounding volume
    float tp = (maxHei-ro.y)/rd.y; if( tp>0.0 ) tmax = min( tmax, tp );

    float res = 1.0;
    float t = mint;
    for( int i=0; i<16; i++ )
    {
    float h = scene_sdf( ro + rd*t );
        res = min( res, 8.0*h/t );
        t += clamp( h, 0.02, 0.10 );
        if( res<0.005 || t>tmax ) break;
    }
    return clamp( res, 0.0, 1.0 );
}

float calcAO( in vec3 pos, in vec3 nor )
{
  float occ = 0.0;
    float sca = 1.0;
    for( int i=0; i<5; i++ )
    {
        float hr = 0.01 + 0.12*float(i)/4.0;
        vec3 aopos =  nor * hr + pos;
        float dd = scene_sdf( aopos );
        occ += -(dd-hr)*sca;
        sca *= 0.95;
    }
    return clamp( 1.0 - 3.0*occ, 0.0, 1.0 ) * (0.5+0.5*nor.y);
}

vec3 render(in vec3 ro, in vec3 rd, in vec3 rdx, in vec3 rdy)
{
    vec3 col = vec3(0.7, 0.7, 0.9) - max(rd.y,0.0)*0.3;
    float t = march_ray(ro,rd);
    vec3 pos = ro + t*rd;
    vec3 nor = estimate_normal(pos);
    vec3 ref = reflect( rd, nor );

    if (t > MAX_DIST - EPS) {
      //return vec3(0.0);
    }

    col = 0.2 + 0.18*sin( 7.6*2.0 + vec3(0.0,0.5,1.0) );
    float occ = calcAO(pos, nor);
    vec3  lig = normalize( vec3(-0.5, 0.4, -0.6) );
    vec3  hal = normalize( lig-rd );
    float amb = sqrt(clamp( 0.5+0.5*nor.y, 0.0, 1.0 ));
    float dif = clamp( dot( nor, lig ), 0.0, 1.0 );
    float bac = clamp( dot( nor, normalize(vec3(-lig.x,0.0,-lig.z))), 0.0, 1.0 )*clamp( 1.0-pos.y,0.0,1.0);
    float dom = smoothstep( -0.2, 0.2, ref.y );
    float fre = pow( clamp(1.0+dot(nor,rd),0.0,1.0), 2.0 );

    //dif *= calcSoftshadow( pos, lig, 0.02, 2.5 );
    //dom *= calcSoftshadow( pos, ref, 0.02, 2.5 );
    float spe = pow( clamp( dot( nor, hal ), 0.0, 1.0 ),16.0) *
        dif *
        (0.04 + 0.96*pow( clamp(1.0+dot(hal,rd),0.0,1.0), 5.0 ));

    vec3 lin = vec3(0.0);
    lin += 3.80*dif*vec3(1.30,1.00,0.70);
    lin += 0.55*amb*vec3(0.40,0.60,1.15)*occ;
    lin += 0.85*dom*vec3(0.40,0.60,1.30)*occ;
    lin += 0.55*bac*vec3(0.25,0.25,0.25)*occ;
    lin += 0.25*fre*vec3(1.00,1.00,1.00)*occ;
    col = col*lin;
    col += 7.00*spe*vec3(1.10,0.90,0.70);

    col = mix( col, vec3(0.7,0.7,0.9), 1.0-exp( -0.0001*t*t*t ) );

  return vec3( clamp(col,0.0,1.0) );
}

mat3 get_view_matrix(in vec3 eye, in vec3 target, float up_angle) {
    vec3 w = normalize(target-eye);
    vec3 up = vec3(sin(up_angle), cos(up_angle),0.0);
    vec3 u = normalize( cross(w,up) );
    vec3 v = cross(u,w);
    return mat3( u, v, w );
}

vec3 get_ray_dir(float fov, vec2 size, vec2 fragCoord) {
  vec2 xy = fragCoord - size / 2.0;
  float z = size.y / tan(radians(fov) / 2.0);
  return normalize(vec3(xy, -z));
}


void mainImage( out vec4 fragColor, in vec2 fragCoord )
{
  float time = 15.0 + iTime*2.5;
  vec3 viewDir = get_ray_dir(90.0, iResolution.xy, fragCoord);
    // camera
    vec3 ray_origin = vec3( 0.5+3.5*cos(0.1*time), 0.0, -0.5+3.5*sin(0.1*time) );
    ray_origin = vec3(0, 0.0,cos(0.4*time));
    vec3 target = vec3( 0.0, 0.0, -1.0 );
    // camera-to-world transformation
    mat3 c2w = get_view_matrix( ray_origin, target, 0.0 );

    vec3 result = vec3(0.0);
#if AA>1
    for(int m = 0; m < AA; m++ )
    for(int n = 0; n < AA; n++ )
    {
        // pixel coordinates
        vec2 o = vec2(float(m),float(n)) / float(AA) - 0.5;
#else
        vec2 o = vec2(0.0);
#endif
        vec3 view_dir = get_ray_dir(94.0, iResolution.xy, fragCoord+o);

        // ray direction
        //vec3 ray_dir = c2w * normalize( vec3(p,2.0) );
        vec3 ray_dir = c2w * view_dir;


         // ray differentials
        vec2 px = (-iResolution.xy+2.0*(fragCoord.xy+vec2(1.0,0.0)))/iResolution.y;
        vec2 py = (-iResolution.xy+2.0*(fragCoord.xy+vec2(0.0,1.0)))/iResolution.y;
        vec3 rdx = c2w * normalize( vec3(px,2.0) );
        vec3 rdy = c2w * normalize( vec3(py,2.0) );

        // render
        vec3 col = render( ray_origin, ray_dir, rdx, rdy );

    // gamma
        col = pow( col, vec3(0.4545) );

        result += col;
#if AA>1
    }
    result /= float(AA*AA);
#endif


    fragColor = vec4( result, 1.0 );
}
