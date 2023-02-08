class ClickAndHold {
    /**
     * @param {EventTarget} target The HTML element to apply the event to
     * @param {Function} callback The function to run once the target is clicked and held
     */
    constructor(target, callback) {
      this.target = target;
      this.callback = callback;
      this.isHeld = false;
      this.activeHoldIntervalId = null;
      this.step = 0;
      ["pointerdown, mousedown, touchstart"].forEach(type => {
        this.target.addEventListener(type, this._onHoldStart.bind(this));
      })
      this.target.addEventListener("pointerdown", this._onHoldStart.bind(this));
      ["pointerup", "mouseleave", "mouseout", "touchend", "touchcancel"].forEach(type => {
        this.target.addEventListener(type, this._onHoldEnd.bind(this));
      });
    }

    _onHoldStart() {
      this.isHeld = true;
      if (this.step == 0){
        this.callback(this.step)
      }
      this.activeHoldIntervalId = setInterval(() => {
        if (this.isHeld) {
          if (this.step < 20) {
            this.step++;
          }
          this.callback(this.step/100);
        }
      }, 20);
    }

    _onHoldEnd() {
      this.isHeld = false;
      clearTimeout(this.activeHoldIntervalId);
      this.step = 0;
    }

    static apply(target, callback) {
      new ClickAndHold(target, callback);
    }
  }

  var zoom = 1;
