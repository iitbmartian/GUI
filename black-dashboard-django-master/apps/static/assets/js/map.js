// import '/static/assets/node_modules/ol/ol.css';
// import GeoJSON from '/static/assets/node_modules/ol/format/GeoJSON';
// import Map from '/static/assets/node_modules/ol/Map';
// import View from '/static/assets/node_modules/ol/View';
// import {Circle as CircleStyle, Fill, Stroke, Style} from '/static/assets/node_modules/ol/style';
// import {OSM, Vector as VectorSource} from '/static/assets/node_modules/ol/source';
// import {Tile as TileLayer, Vector as VectorLayer} from '/static/assets/node_modules/ol/layer';

/** @type {VectorSource<import("../src/ol/geom/SimpleGeometry.js").default>} */
const source = new ol.source.Vector({
  url: '/static/assets/base_station.geojson',
  format: new ol.format.GeoJSON(),
});
const style = new ol.style.Style({
  fill: new ol.style.Fill({
    color: 'rgba(255, 255, 255, 0.6)',
  }),
  stroke: new ol.style.Stroke({
    color: '#319FD3',
    width: 1,
  }),
  image: new ol.style.Circle({
    radius: 5,
    fill: new ol.style.Fill({
      color: 'rgba(255, 255, 255, 0.6)',
    }),
    stroke: new ol.style.Stroke({
      color: '#319FD3',
      width: 1,
    }),
  }),
});

// 
// 
class Drag extends ol.interaction.Pointer {
  constructor() {
    super({
      handleDownEvent: handleDownEvent,
      handleDragEvent: handleDragEvent,
      handleMoveEvent: handleMoveEvent,
      handleUpEvent: handleUpEvent,
    });

    /**
     * @type {import("../src/ol/coordinate.js").Coordinate}
     * @private
     */
    this.coordinate_ = null;

    /**
     * @type {string|undefined}
     * @private
     */
    this.cursor_ = 'pointer';

    /**
     * @type {Feature}
     * @private
     */
    this.feature_ = null;

    /**
     * @type {string|undefined}
     * @private
     */
    this.previousCursor_ = undefined;
  }
}

/**
 * @param {import("../src/ol/MapBrowserEvent.js").default} evt Map browser event.
 * @return {boolean} `true` to start the drag sequence.
 */
function handleDownEvent(evt) {
  const map = evt.map;

  const feature = map.forEachFeatureAtPixel(evt.pixel, function (feature) {
    return feature;
  });

  if (feature) {
    this.coordinate_ = evt.coordinate;
    this.feature_ = feature;
  }

  return !!feature;
}

/**
 * @param {import("../src/ol/MapBrowserEvent.js").default} evt Map browser event.
 */
function handleDragEvent(evt) {
  const deltaX = evt.coordinate[0] - this.coordinate_[0];
  const deltaY = evt.coordinate[1] - this.coordinate_[1];

  const geometry = this.feature_.getGeometry();
  geometry.translate(deltaX, deltaY);

  this.coordinate_[0] = evt.coordinate[0];
  this.coordinate_[1] = evt.coordinate[1];
}

/**
 * @param {import("../src/ol/MapBrowserEvent.js").default} evt Event.
 */
function handleMoveEvent(evt) {
  if (this.cursor_) {
    const map = evt.map;
    const feature = map.forEachFeatureAtPixel(evt.pixel, function (feature) {
      return feature;
    });
    const element = evt.map.getTargetElement();
    if (feature) {
      if (element.style.cursor != this.cursor_) {
        this.previousCursor_ = element.style.cursor;
        element.style.cursor = this.cursor_;
      }
    } else if (this.previousCursor_ !== undefined) {
      element.style.cursor = this.previousCursor_;
      this.previousCursor_ = undefined;
    }
  }
}

/**
 * @return {boolean} `false` to stop the drag sequence.
 */
function handleUpEvent() {
  this.coordinate_ = null;
  this.feature_ = null;
  return false;
}

const pointFeature = new ol.Feature(new ol.geom.Point(ol.proj.fromLonLat([72.918584, 19.130790])));

// const lineFeature = new Feature(
//   new LineString([
//     [-1e7, 1e6],
//     [-1e6, 3e6],
//   ])
// );
// 
// 

const circleFeature = new ol.Feature({
  geometry: new ol.geom.Circle(ol.proj.fromLonLat([72.918584, 19.130790]), 5),
});
circleFeature.setStyle(
  new ol.style.Style({
    renderer(coordinates, state) {
      const [[x, y], [x1, y1]] = coordinates;
      const ctx = state.context;
      const dx = x1 - x;
      const dy = y1 - y;
      const radius = Math.sqrt(dx * dx + dy * dy);

      const innerRadius = 0;
      const outerRadius = radius * 1.4;

      const gradient = ctx.createRadialGradient(
        x,
        y,
        innerRadius,
        x,
        y,
        outerRadius
      );
      gradient.addColorStop(0, 'rgba(255,0,0,0)');
      gradient.addColorStop(0.6, 'rgba(255,0,0,0.2)');
      gradient.addColorStop(1, 'rgba(255,0,0,0.8)');
      ctx.beginPath();
      ctx.arc(x, y, radius, 0, 2 * Math.PI, true);
      ctx.fillStyle = gradient;
      ctx.fill();

      ctx.arc(x, y, radius, 0, 2 * Math.PI, true);
      ctx.strokeStyle = 'rgba(255,0,0,1)';
      ctx.stroke();
    },
  })
);
const vectorLayer = new ol.layer.Vector({
  source: source,
  style: style,
});
const view = new ol.View({
  center: ol.proj.fromLonLat([72.918584, 19.130790]),
  zoom: 18
});
const map = new ol.Map({  
  interactions: ol.interaction.defaults().extend([new Drag()]),
  layers: [
    new ol.layer.Tile({
      source: new ol.source.OSM(),
    }),
    vectorLayer,
    new ol.layer.Vector({
      source: new ol.source.Vector({
        features: [circleFeature],
      }),
    }),
     new ol.layer.Vector({
      source: new ol.source.Vector({
        features: [pointFeature],
      }),
      style: new ol.style.Style({
        image: new ol.style.Icon({
          anchor: [0.5, 1.0],
          anchorXUnits: 'fraction',
          anchorYUnits: 'fraction',
          opacity: 0.95,
          src: '/static/assets/img/pin.png',
          scale: 0.05,
        }),
        stroke: new ol.style.Stroke({
          width: 3,
          color: [255, 0, 0, 1],
        }),
        fill: new ol.style.Fill({
          color: [0, 0, 255, 0.6],
        }),
      }),
    }),
  ],
  target: 'map',
  view: view,
});

const zoomtobasestation = document.getElementById('zoomtobasestation');
zoomtobasestation.addEventListener(
  'click',
  function () {
    const feature = source.getFeatures()[0];
    const polygon = feature.getGeometry();
    view.fit(polygon, {padding: [17, 5, 3, 15], zoom:18, duration: 100});
  },
  false
);

const zoomtorover = document.getElementById('zoomtorover');
zoomtorover.addEventListener(
  'click',
  function () {
    const feature = source.getFeatures()[1];
    const point = feature.getGeometry();
    view.fit(point, {padding: [150, 50, 50, 150], maxZoom:18, duration: 100});
  },
  false
);

const centerrover = document.getElementById('centerrover');
centerrover.addEventListener(
  'click',
  function () {
    const feature = source.getFeatures()[1];
    const point = feature.getGeometry();
    const size = map.getSize();
    view.animate({center: point.getCoordinates(), zoom:18, duration:100} );
  },
  false
);
