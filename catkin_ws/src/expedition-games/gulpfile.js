/*
 * gulpfile.js
 *    Defines the gulp tasks
 */


// Load Plugins
//    be sure to npm install any new plugins
//        $ npm install --save-dev <plugin>
var gulp = require('gulp'),
    browserify = require('browserify'),
    source = require('vinyl-source-stream'),
    sourcemaps = require('gulp-sourcemaps');

var bundle_options = {
      entries: ['./scripts/index.js'],
      standalone: 'SARLIB'
};

/*
 * build
 *    Concats all JS games into a single minified JS file
 */
gulp.task('build', function() {
  return browserify(bundle_options)
        .bundle()
        .pipe(source('main.js'))
        //.pipe(sourcemaps.init({loadmaps: true}))
        //.pipe(sourcemaps.write())
        .pipe(gulp.dest('scripts/build'));
});


gulp.task('watch', function() {
  gulp.watch(['scripts/**/*.js', '!scripts/build/**/*.js'], ['build']);
});

// Default
gulp.task('default', ['watch']);
