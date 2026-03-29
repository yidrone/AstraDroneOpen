/**
 * TrackballControls.js (ESM) - Proper sphere projection rotation (multi-axis)
 * Mouse:
 *  - Left drag: rotate (true trackball)
 *  - Wheel: zoom
 *  - Right drag: pan
 * Touch:
 *  - 1 finger: rotate
 *  - 2 fingers pinch: zoom
 *  - 2 fingers move: pan
 */
import { EventDispatcher, MOUSE, Quaternion, Vector2, Vector3 } from 'three';

class TrackballControls extends EventDispatcher {
  constructor(object, domElement) {
    super();

    this.object = object;
    this.domElement = domElement;

    this.enabled = true;

    this.rotateSpeed = 2.8;
    this.zoomSpeed = 1.2;
    this.panSpeed = 0.8;

    this.staticMoving = false;
    this.dynamicDampingFactor = 0.12;

    this.minDistance = 0;
    this.maxDistance = Infinity;

    this.noRotate = false;
    this.noZoom = false;
    this.noPan = false;

    this.target = new Vector3();

    const EPS = 1e-6;

    const STATE = { NONE: -1, ROTATE: 0, PAN: 1, TOUCH_ZOOM_PAN: 2 };
    let state = STATE.NONE;

    const lastPosition = new Vector3();

    // ===== rotate trackball (3D) =====
    const _vPrev = new Vector3();
    const _vCurr = new Vector3();
    const _axis = new Vector3();
    const _quat = new Quaternion();
    let _lastAngle = 0;
    const _lastAxis = new Vector3();

    // ===== pan =====
    const _panStart = new Vector2();
    const _panEnd = new Vector2();

    // ===== touch zoom/pan =====
    const pointers = new Map(); // pointerId -> {x,y}
    const _t1 = new Vector2(), _t2 = new Vector2();
    const _t1Prev = new Vector2(), _t2Prev = new Vector2();
    let _prevPinchDist = 0;

    const rect = () => this.domElement.getBoundingClientRect();

    const getEye = () => new Vector3().subVectors(this.object.position, this.target);

    const projectOnTrackball = (clientX, clientY, outV3) => {
      // map to -1..1
      const r = rect();
      const x = ( (clientX - r.left) / r.width ) * 2 - 1;
      const y = - ( (clientY - r.top) / r.height ) * 2 + 1;

      // virtual sphere radius = 1
      const z2 = 1 - x*x - y*y;
      const z = z2 > 0 ? Math.sqrt(z2) : 0;

      outV3.set(x, y, z).normalize();
    };

    const applyZoomFactor = (factor) => {
      if (this.noZoom) return;
      if (!(factor > 0)) return;

      const eye = getEye().multiplyScalar(factor);
      const len = eye.length();

      if (len > this.maxDistance) eye.setLength(this.maxDistance);
      if (len < this.minDistance) eye.setLength(this.minDistance);

      this.object.position.copy(this.target).add(eye);
    };

    const applyPanDelta = (dx, dy) => {
      if (this.noPan) return;

      const eye = getEye();
      const dist = eye.length();
      const scale = dist * this.panSpeed;

      const right = new Vector3().copy(eye).cross(this.object.up).normalize();
      const up = new Vector3().copy(this.object.up).normalize();

      const pan = new Vector3()
        .addScaledVector(right, dx * scale)
        .addScaledVector(up, dy * scale);

      this.object.position.add(pan);
      this.target.add(pan);
    };

    const rotateCamera = () => {
      // compute angle & axis in trackball space
      const dot = Math.min(1, Math.max(-1, _vPrev.dot(_vCurr)));
      const angle = Math.acos(dot);

      if (angle > EPS) {
        _axis.crossVectors(_vPrev, _vCurr).normalize();

        // IMPORTANT:
        // axis is in "screen space" trackball coords, but we want it in world coords.
        // Build basis from camera: right, up, forward
        const eye = getEye();
        const forward = eye.clone().normalize();                 // from target to camera
        const camUp = this.object.up.clone().normalize();
        const camRight = new Vector3().crossVectors(camUp, forward).normalize();

        // convert axis from (x=right, y=up, z=forward) basis into world axis:
        const worldAxis = new Vector3()
          .addScaledVector(camRight, _axis.x)
          .addScaledVector(camUp, _axis.y)
          .addScaledVector(forward, _axis.z)
          .normalize();

        const a = angle * this.rotateSpeed;
        _quat.setFromAxisAngle(worldAxis, -a);

        // rotate eye vector around target
        eye.applyQuaternion(_quat);
        this.object.up.applyQuaternion(_quat);

        this.object.position.copy(this.target).add(eye);

        _lastAxis.copy(worldAxis);
        _lastAngle = a;
      } else if (!this.staticMoving && _lastAngle > EPS) {
        const eye = getEye();
        _lastAngle *= Math.sqrt(1.0 - this.dynamicDampingFactor);
        _quat.setFromAxisAngle(_lastAxis, _lastAngle);
        eye.applyQuaternion(_quat);
        this.object.up.applyQuaternion(_quat);
        this.object.position.copy(this.target).add(eye);
      }

      _vPrev.copy(_vCurr);
    };

    const checkDistances = () => {
      if (this.noZoom && this.noPan) return;

      const eye = getEye();
      const len = eye.length();

      if (len > this.maxDistance) {
        eye.setLength(this.maxDistance);
        this.object.position.copy(this.target).add(eye);
      }
      if (len < this.minDistance) {
        eye.setLength(this.minDistance);
        this.object.position.copy(this.target).add(eye);
      }
    };

    this.update = () => {
      if (!this.enabled) return;

      if (!this.noRotate && state === STATE.ROTATE) rotateCamera();

      if (!this.noPan && state === STATE.PAN) {
        const delta = _panEnd.clone().sub(_panStart);
        if (delta.lengthSq() > EPS) {
          applyPanDelta(delta.x, delta.y);
          if (this.staticMoving) {
            _panStart.copy(_panEnd);
          } else {
            _panStart.add(delta.multiplyScalar(this.dynamicDampingFactor));
          }
        }
      }

      checkDistances();
      this.object.lookAt(this.target);

      if (lastPosition.distanceToSquared(this.object.position) > EPS) {
        this.dispatchEvent({ type: 'change' });
        lastPosition.copy(this.object.position);
      }
    };

    // ===== pointer events =====
    const setPointer = (e) => pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
    const delPointer = (e) => pointers.delete(e.pointerId);

    const pointerdown = (e) => {
      if (!this.enabled) return;
      this.domElement.setPointerCapture?.(e.pointerId);
      setPointer(e);

      if (pointers.size === 1) {
        if (e.pointerType === 'mouse' && e.button === MOUSE.RIGHT) {
          state = STATE.PAN;
          const r = rect();
          _panStart.set((e.clientX - r.left) / r.width, (e.clientY - r.top) / r.height);
          _panEnd.copy(_panStart);
        } else {
          state = STATE.ROTATE;
          projectOnTrackball(e.clientX, e.clientY, _vCurr);
          _vPrev.copy(_vCurr);
        }
      } else if (pointers.size === 2) {
        state = STATE.TOUCH_ZOOM_PAN;
        const arr = Array.from(pointers.values());
        _t1.set(arr[0].x, arr[0].y);
        _t2.set(arr[1].x, arr[1].y);
        _t1Prev.copy(_t1);
        _t2Prev.copy(_t2);
        _prevPinchDist = _t1.distanceTo(_t2);
      }

      e.preventDefault();
      e.stopPropagation();
    };

    const pointermove = (e) => {
      if (!this.enabled) return;
      if (!pointers.has(e.pointerId)) return;

      setPointer(e);

      if (state === STATE.ROTATE && pointers.size === 1) {
        projectOnTrackball(e.clientX, e.clientY, _vCurr);
      } else if (state === STATE.PAN && pointers.size === 1) {
        const r = rect();
        _panEnd.set((e.clientX - r.left) / r.width, (e.clientY - r.top) / r.height);
      } else if (state === STATE.TOUCH_ZOOM_PAN && pointers.size === 2) {
        const arr = Array.from(pointers.values());
        _t1.set(arr[0].x, arr[0].y);
        _t2.set(arr[1].x, arr[1].y);

        // pinch zoom
        const dist = _t1.distanceTo(_t2);
        if (_prevPinchDist > 0 && dist > 0) {
          const r = rect();
          const delta = (_prevPinchDist - dist) / Math.max(1, r.width);
          const factor = 1.0 + delta * this.zoomSpeed * 2.0;
          applyZoomFactor(Math.max(0.2, Math.min(5.0, factor)));
        }
        _prevPinchDist = dist;

        // two-finger pan (midpoint)
        const mid = _t1.clone().add(_t2).multiplyScalar(0.5);
        const midPrev = _t1Prev.clone().add(_t2Prev).multiplyScalar(0.5);
        const r = rect();
        const dx = (mid.x - midPrev.x) / Math.max(1, r.width);
        const dy = (mid.y - midPrev.y) / Math.max(1, r.height);
        applyPanDelta(dx, dy);

        _t1Prev.copy(_t1);
        _t2Prev.copy(_t2);
      }

      e.preventDefault();
      e.stopPropagation();
    };

    const pointerup = (e) => {
      if (!this.enabled) return;
      delPointer(e);

      if (pointers.size === 0) {
        state = STATE.NONE;
      } else if (pointers.size === 1) {
        state = STATE.ROTATE;
        const only = Array.from(pointers.values())[0];
        projectOnTrackball(only.x, only.y, _vCurr);
        _vPrev.copy(_vCurr);
      }

      e.preventDefault();
      e.stopPropagation();
    };

    const wheel = (e) => {
      if (!this.enabled || this.noZoom) return;
      e.preventDefault();
      const factor = (e.deltaY > 0) ? 1.10 : 0.90;
      applyZoomFactor(factor);
    };

    const contextmenu = (e) => e.preventDefault();

    this.domElement.addEventListener('contextmenu', contextmenu, false);
    this.domElement.addEventListener('pointerdown', pointerdown, { passive: false });
    this.domElement.addEventListener('pointermove', pointermove, { passive: false });
    this.domElement.addEventListener('pointerup', pointerup, { passive: false });
    this.domElement.addEventListener('pointercancel', pointerup, { passive: false });
    this.domElement.addEventListener('wheel', wheel, { passive: false });

    this.dispose = () => {
      this.domElement.removeEventListener('contextmenu', contextmenu, false);
      this.domElement.removeEventListener('pointerdown', pointerdown);
      this.domElement.removeEventListener('pointermove', pointermove);
      this.domElement.removeEventListener('pointerup', pointerup);
      this.domElement.removeEventListener('pointercancel', pointerup);
      this.domElement.removeEventListener('wheel', wheel);
    };
  }
}

export { TrackballControls };
