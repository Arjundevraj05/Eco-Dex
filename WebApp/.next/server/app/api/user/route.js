/*
 * ATTENTION: An "eval-source-map" devtool has been used.
 * This devtool is neither made for production nor for readable output files.
 * It uses "eval()" calls to create a separate source file with attached SourceMaps in the browser devtools.
 * If you are trying to read the output file, select a different devtool (https://webpack.js.org/configuration/devtool/)
 * or disable the default devtool with "devtool: false".
 * If you are looking for production-ready output files, see mode: "production" (https://webpack.js.org/configuration/mode/).
 */
(() => {
var exports = {};
exports.id = "app/api/user/route";
exports.ids = ["app/api/user/route"];
exports.modules = {

/***/ "next/dist/compiled/next-server/app-page.runtime.dev.js":
/*!*************************************************************************!*\
  !*** external "next/dist/compiled/next-server/app-page.runtime.dev.js" ***!
  \*************************************************************************/
/***/ ((module) => {

"use strict";
module.exports = require("next/dist/compiled/next-server/app-page.runtime.dev.js");

/***/ }),

/***/ "next/dist/compiled/next-server/app-route.runtime.dev.js":
/*!**************************************************************************!*\
  !*** external "next/dist/compiled/next-server/app-route.runtime.dev.js" ***!
  \**************************************************************************/
/***/ ((module) => {

"use strict";
module.exports = require("next/dist/compiled/next-server/app-route.runtime.dev.js");

/***/ }),

/***/ "../app-render/after-task-async-storage.external":
/*!***********************************************************************************!*\
  !*** external "next/dist/server/app-render/after-task-async-storage.external.js" ***!
  \***********************************************************************************/
/***/ ((module) => {

"use strict";
module.exports = require("next/dist/server/app-render/after-task-async-storage.external.js");

/***/ }),

/***/ "../app-render/work-async-storage.external":
/*!*****************************************************************************!*\
  !*** external "next/dist/server/app-render/work-async-storage.external.js" ***!
  \*****************************************************************************/
/***/ ((module) => {

"use strict";
module.exports = require("next/dist/server/app-render/work-async-storage.external.js");

/***/ }),

/***/ "./work-unit-async-storage.external":
/*!**********************************************************************************!*\
  !*** external "next/dist/server/app-render/work-unit-async-storage.external.js" ***!
  \**********************************************************************************/
/***/ ((module) => {

"use strict";
module.exports = require("next/dist/server/app-render/work-unit-async-storage.external.js");

/***/ }),

/***/ "buffer":
/*!*************************!*\
  !*** external "buffer" ***!
  \*************************/
/***/ ((module) => {

"use strict";
module.exports = require("buffer");

/***/ }),

/***/ "crypto":
/*!*************************!*\
  !*** external "crypto" ***!
  \*************************/
/***/ ((module) => {

"use strict";
module.exports = require("crypto");

/***/ }),

/***/ "stream":
/*!*************************!*\
  !*** external "stream" ***!
  \*************************/
/***/ ((module) => {

"use strict";
module.exports = require("stream");

/***/ }),

/***/ "util":
/*!***********************!*\
  !*** external "util" ***!
  \***********************/
/***/ ((module) => {

"use strict";
module.exports = require("util");

/***/ }),

/***/ "(rsc)/./node_modules/next/dist/build/webpack/loaders/next-app-loader/index.js?name=app%2Fapi%2Fuser%2Froute&page=%2Fapi%2Fuser%2Froute&appPaths=&pagePath=private-next-app-dir%2Fapi%2Fuser%2Froute.ts&appDir=D%3A%5CApplications%5CEco-Dex%5CWebApp%5Capp&pageExtensions=tsx&pageExtensions=ts&pageExtensions=jsx&pageExtensions=js&rootDir=D%3A%5CApplications%5CEco-Dex%5CWebApp&isDev=true&tsconfigPath=tsconfig.json&basePath=&assetPrefix=&nextConfigOutput=&preferredRegion=&middlewareConfig=e30%3D!":
/*!**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************!*\
  !*** ./node_modules/next/dist/build/webpack/loaders/next-app-loader/index.js?name=app%2Fapi%2Fuser%2Froute&page=%2Fapi%2Fuser%2Froute&appPaths=&pagePath=private-next-app-dir%2Fapi%2Fuser%2Froute.ts&appDir=D%3A%5CApplications%5CEco-Dex%5CWebApp%5Capp&pageExtensions=tsx&pageExtensions=ts&pageExtensions=jsx&pageExtensions=js&rootDir=D%3A%5CApplications%5CEco-Dex%5CWebApp&isDev=true&tsconfigPath=tsconfig.json&basePath=&assetPrefix=&nextConfigOutput=&preferredRegion=&middlewareConfig=e30%3D! ***!
  \**************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
eval("__webpack_require__.r(__webpack_exports__);\n/* harmony export */ __webpack_require__.d(__webpack_exports__, {\n/* harmony export */   patchFetch: () => (/* binding */ patchFetch),\n/* harmony export */   routeModule: () => (/* binding */ routeModule),\n/* harmony export */   serverHooks: () => (/* binding */ serverHooks),\n/* harmony export */   workAsyncStorage: () => (/* binding */ workAsyncStorage),\n/* harmony export */   workUnitAsyncStorage: () => (/* binding */ workUnitAsyncStorage)\n/* harmony export */ });\n/* harmony import */ var next_dist_server_route_modules_app_route_module_compiled__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! next/dist/server/route-modules/app-route/module.compiled */ \"(rsc)/./node_modules/next/dist/server/route-modules/app-route/module.compiled.js\");\n/* harmony import */ var next_dist_server_route_modules_app_route_module_compiled__WEBPACK_IMPORTED_MODULE_0___default = /*#__PURE__*/__webpack_require__.n(next_dist_server_route_modules_app_route_module_compiled__WEBPACK_IMPORTED_MODULE_0__);\n/* harmony import */ var next_dist_server_route_kind__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! next/dist/server/route-kind */ \"(rsc)/./node_modules/next/dist/server/route-kind.js\");\n/* harmony import */ var next_dist_server_lib_patch_fetch__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! next/dist/server/lib/patch-fetch */ \"(rsc)/./node_modules/next/dist/server/lib/patch-fetch.js\");\n/* harmony import */ var next_dist_server_lib_patch_fetch__WEBPACK_IMPORTED_MODULE_2___default = /*#__PURE__*/__webpack_require__.n(next_dist_server_lib_patch_fetch__WEBPACK_IMPORTED_MODULE_2__);\n/* harmony import */ var D_Applications_Eco_Dex_WebApp_app_api_user_route_ts__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! ./app/api/user/route.ts */ \"(rsc)/./app/api/user/route.ts\");\n\n\n\n\n// We inject the nextConfigOutput here so that we can use them in the route\n// module.\nconst nextConfigOutput = \"\"\nconst routeModule = new next_dist_server_route_modules_app_route_module_compiled__WEBPACK_IMPORTED_MODULE_0__.AppRouteRouteModule({\n    definition: {\n        kind: next_dist_server_route_kind__WEBPACK_IMPORTED_MODULE_1__.RouteKind.APP_ROUTE,\n        page: \"/api/user/route\",\n        pathname: \"/api/user\",\n        filename: \"route\",\n        bundlePath: \"app/api/user/route\"\n    },\n    resolvedPagePath: \"D:\\\\Applications\\\\Eco-Dex\\\\WebApp\\\\app\\\\api\\\\user\\\\route.ts\",\n    nextConfigOutput,\n    userland: D_Applications_Eco_Dex_WebApp_app_api_user_route_ts__WEBPACK_IMPORTED_MODULE_3__\n});\n// Pull out the exports that we need to expose from the module. This should\n// be eliminated when we've moved the other routes to the new format. These\n// are used to hook into the route.\nconst { workAsyncStorage, workUnitAsyncStorage, serverHooks } = routeModule;\nfunction patchFetch() {\n    return (0,next_dist_server_lib_patch_fetch__WEBPACK_IMPORTED_MODULE_2__.patchFetch)({\n        workAsyncStorage,\n        workUnitAsyncStorage\n    });\n}\n\n\n//# sourceMappingURL=app-route.js.map//# sourceURL=[module]\n//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiKHJzYykvLi9ub2RlX21vZHVsZXMvbmV4dC9kaXN0L2J1aWxkL3dlYnBhY2svbG9hZGVycy9uZXh0LWFwcC1sb2FkZXIvaW5kZXguanM/bmFtZT1hcHAlMkZhcGklMkZ1c2VyJTJGcm91dGUmcGFnZT0lMkZhcGklMkZ1c2VyJTJGcm91dGUmYXBwUGF0aHM9JnBhZ2VQYXRoPXByaXZhdGUtbmV4dC1hcHAtZGlyJTJGYXBpJTJGdXNlciUyRnJvdXRlLnRzJmFwcERpcj1EJTNBJTVDQXBwbGljYXRpb25zJTVDRWNvLURleCU1Q1dlYkFwcCU1Q2FwcCZwYWdlRXh0ZW5zaW9ucz10c3gmcGFnZUV4dGVuc2lvbnM9dHMmcGFnZUV4dGVuc2lvbnM9anN4JnBhZ2VFeHRlbnNpb25zPWpzJnJvb3REaXI9RCUzQSU1Q0FwcGxpY2F0aW9ucyU1Q0Vjby1EZXglNUNXZWJBcHAmaXNEZXY9dHJ1ZSZ0c2NvbmZpZ1BhdGg9dHNjb25maWcuanNvbiZiYXNlUGF0aD0mYXNzZXRQcmVmaXg9Jm5leHRDb25maWdPdXRwdXQ9JnByZWZlcnJlZFJlZ2lvbj0mbWlkZGxld2FyZUNvbmZpZz1lMzAlM0QhIiwibWFwcGluZ3MiOiI7Ozs7Ozs7Ozs7Ozs7O0FBQStGO0FBQ3ZDO0FBQ3FCO0FBQ1c7QUFDeEY7QUFDQTtBQUNBO0FBQ0Esd0JBQXdCLHlHQUFtQjtBQUMzQztBQUNBLGNBQWMsa0VBQVM7QUFDdkI7QUFDQTtBQUNBO0FBQ0E7QUFDQSxLQUFLO0FBQ0w7QUFDQTtBQUNBLFlBQVk7QUFDWixDQUFDO0FBQ0Q7QUFDQTtBQUNBO0FBQ0EsUUFBUSxzREFBc0Q7QUFDOUQ7QUFDQSxXQUFXLDRFQUFXO0FBQ3RCO0FBQ0E7QUFDQSxLQUFLO0FBQ0w7QUFDMEY7O0FBRTFGIiwic291cmNlcyI6WyIiXSwic291cmNlc0NvbnRlbnQiOlsiaW1wb3J0IHsgQXBwUm91dGVSb3V0ZU1vZHVsZSB9IGZyb20gXCJuZXh0L2Rpc3Qvc2VydmVyL3JvdXRlLW1vZHVsZXMvYXBwLXJvdXRlL21vZHVsZS5jb21waWxlZFwiO1xuaW1wb3J0IHsgUm91dGVLaW5kIH0gZnJvbSBcIm5leHQvZGlzdC9zZXJ2ZXIvcm91dGUta2luZFwiO1xuaW1wb3J0IHsgcGF0Y2hGZXRjaCBhcyBfcGF0Y2hGZXRjaCB9IGZyb20gXCJuZXh0L2Rpc3Qvc2VydmVyL2xpYi9wYXRjaC1mZXRjaFwiO1xuaW1wb3J0ICogYXMgdXNlcmxhbmQgZnJvbSBcIkQ6XFxcXEFwcGxpY2F0aW9uc1xcXFxFY28tRGV4XFxcXFdlYkFwcFxcXFxhcHBcXFxcYXBpXFxcXHVzZXJcXFxccm91dGUudHNcIjtcbi8vIFdlIGluamVjdCB0aGUgbmV4dENvbmZpZ091dHB1dCBoZXJlIHNvIHRoYXQgd2UgY2FuIHVzZSB0aGVtIGluIHRoZSByb3V0ZVxuLy8gbW9kdWxlLlxuY29uc3QgbmV4dENvbmZpZ091dHB1dCA9IFwiXCJcbmNvbnN0IHJvdXRlTW9kdWxlID0gbmV3IEFwcFJvdXRlUm91dGVNb2R1bGUoe1xuICAgIGRlZmluaXRpb246IHtcbiAgICAgICAga2luZDogUm91dGVLaW5kLkFQUF9ST1VURSxcbiAgICAgICAgcGFnZTogXCIvYXBpL3VzZXIvcm91dGVcIixcbiAgICAgICAgcGF0aG5hbWU6IFwiL2FwaS91c2VyXCIsXG4gICAgICAgIGZpbGVuYW1lOiBcInJvdXRlXCIsXG4gICAgICAgIGJ1bmRsZVBhdGg6IFwiYXBwL2FwaS91c2VyL3JvdXRlXCJcbiAgICB9LFxuICAgIHJlc29sdmVkUGFnZVBhdGg6IFwiRDpcXFxcQXBwbGljYXRpb25zXFxcXEVjby1EZXhcXFxcV2ViQXBwXFxcXGFwcFxcXFxhcGlcXFxcdXNlclxcXFxyb3V0ZS50c1wiLFxuICAgIG5leHRDb25maWdPdXRwdXQsXG4gICAgdXNlcmxhbmRcbn0pO1xuLy8gUHVsbCBvdXQgdGhlIGV4cG9ydHMgdGhhdCB3ZSBuZWVkIHRvIGV4cG9zZSBmcm9tIHRoZSBtb2R1bGUuIFRoaXMgc2hvdWxkXG4vLyBiZSBlbGltaW5hdGVkIHdoZW4gd2UndmUgbW92ZWQgdGhlIG90aGVyIHJvdXRlcyB0byB0aGUgbmV3IGZvcm1hdC4gVGhlc2Vcbi8vIGFyZSB1c2VkIHRvIGhvb2sgaW50byB0aGUgcm91dGUuXG5jb25zdCB7IHdvcmtBc3luY1N0b3JhZ2UsIHdvcmtVbml0QXN5bmNTdG9yYWdlLCBzZXJ2ZXJIb29rcyB9ID0gcm91dGVNb2R1bGU7XG5mdW5jdGlvbiBwYXRjaEZldGNoKCkge1xuICAgIHJldHVybiBfcGF0Y2hGZXRjaCh7XG4gICAgICAgIHdvcmtBc3luY1N0b3JhZ2UsXG4gICAgICAgIHdvcmtVbml0QXN5bmNTdG9yYWdlXG4gICAgfSk7XG59XG5leHBvcnQgeyByb3V0ZU1vZHVsZSwgd29ya0FzeW5jU3RvcmFnZSwgd29ya1VuaXRBc3luY1N0b3JhZ2UsIHNlcnZlckhvb2tzLCBwYXRjaEZldGNoLCAgfTtcblxuLy8jIHNvdXJjZU1hcHBpbmdVUkw9YXBwLXJvdXRlLmpzLm1hcCJdLCJuYW1lcyI6W10sImlnbm9yZUxpc3QiOltdLCJzb3VyY2VSb290IjoiIn0=\n//# sourceURL=webpack-internal:///(rsc)/./node_modules/next/dist/build/webpack/loaders/next-app-loader/index.js?name=app%2Fapi%2Fuser%2Froute&page=%2Fapi%2Fuser%2Froute&appPaths=&pagePath=private-next-app-dir%2Fapi%2Fuser%2Froute.ts&appDir=D%3A%5CApplications%5CEco-Dex%5CWebApp%5Capp&pageExtensions=tsx&pageExtensions=ts&pageExtensions=jsx&pageExtensions=js&rootDir=D%3A%5CApplications%5CEco-Dex%5CWebApp&isDev=true&tsconfigPath=tsconfig.json&basePath=&assetPrefix=&nextConfigOutput=&preferredRegion=&middlewareConfig=e30%3D!\n");

/***/ }),

/***/ "(rsc)/./node_modules/next/dist/build/webpack/loaders/next-flight-client-entry-loader.js?server=true!":
/*!******************************************************************************************************!*\
  !*** ./node_modules/next/dist/build/webpack/loaders/next-flight-client-entry-loader.js?server=true! ***!
  \******************************************************************************************************/
/***/ (() => {



/***/ }),

/***/ "(ssr)/./node_modules/next/dist/build/webpack/loaders/next-flight-client-entry-loader.js?server=true!":
/*!******************************************************************************************************!*\
  !*** ./node_modules/next/dist/build/webpack/loaders/next-flight-client-entry-loader.js?server=true! ***!
  \******************************************************************************************************/
/***/ (() => {



/***/ }),

/***/ "(rsc)/./app/api/user/route.ts":
/*!*******************************!*\
  !*** ./app/api/user/route.ts ***!
  \*******************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
eval("__webpack_require__.r(__webpack_exports__);\n/* harmony export */ __webpack_require__.d(__webpack_exports__, {\n/* harmony export */   GET: () => (/* binding */ GET)\n/* harmony export */ });\n/* harmony import */ var next_server__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! next/server */ \"(rsc)/./node_modules/next/dist/api/server.js\");\n/* harmony import */ var jsonwebtoken__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! jsonwebtoken */ \"(rsc)/./node_modules/jsonwebtoken/index.js\");\n/* harmony import */ var jsonwebtoken__WEBPACK_IMPORTED_MODULE_1___default = /*#__PURE__*/__webpack_require__.n(jsonwebtoken__WEBPACK_IMPORTED_MODULE_1__);\n/* harmony import */ var _helper_demoAuth__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! @/helper/demoAuth */ \"(rsc)/./helper/demoAuth.ts\");\n/* harmony import */ var _helper_demoMode__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! @/helper/demoMode */ \"(rsc)/./helper/demoMode.ts\");\n\n\n\n\nconst JWT_SECRET = process.env.JWT_SECRET;\nasync function GET(request) {\n    const authCookie = request.headers.get(\"cookie\")?.split(\"; \").find((c)=>c.startsWith(\"auth_token=\"));\n    if (!authCookie) {\n        if ((0,_helper_demoMode__WEBPACK_IMPORTED_MODULE_3__.isDemoMode)()) {\n            return next_server__WEBPACK_IMPORTED_MODULE_0__.NextResponse.json({\n                username: (0,_helper_demoAuth__WEBPACK_IMPORTED_MODULE_2__.getDemoAuth)().username\n            });\n        }\n        return next_server__WEBPACK_IMPORTED_MODULE_0__.NextResponse.json({\n            error: \"Not authenticated\"\n        }, {\n            status: 401\n        });\n    }\n    const token = authCookie.split(\"=\")[1];\n    if ((0,_helper_demoAuth__WEBPACK_IMPORTED_MODULE_2__.isDemoToken)(token)) {\n        return next_server__WEBPACK_IMPORTED_MODULE_0__.NextResponse.json({\n            username: (0,_helper_demoAuth__WEBPACK_IMPORTED_MODULE_2__.getDemoAuth)().username\n        });\n    }\n    try {\n        const decoded = jsonwebtoken__WEBPACK_IMPORTED_MODULE_1___default().verify(token, JWT_SECRET);\n        return next_server__WEBPACK_IMPORTED_MODULE_0__.NextResponse.json({\n            username: decoded.username\n        });\n    } catch  {\n        return next_server__WEBPACK_IMPORTED_MODULE_0__.NextResponse.json({\n            error: \"Invalid token\"\n        }, {\n            status: 403\n        });\n    }\n}\n//# sourceURL=[module]\n//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiKHJzYykvLi9hcHAvYXBpL3VzZXIvcm91dGUudHMiLCJtYXBwaW5ncyI6Ijs7Ozs7Ozs7O0FBQTJDO0FBQ1o7QUFDOEI7QUFDZDtBQUUvQyxNQUFNSyxhQUFhQyxRQUFRQyxHQUFHLENBQUNGLFVBQVU7QUFFbEMsZUFBZUcsSUFBSUMsT0FBZ0I7SUFDeEMsTUFBTUMsYUFBYUQsUUFBUUUsT0FBTyxDQUMvQkMsR0FBRyxDQUFDLFdBQ0hDLE1BQU0sTUFDUEMsS0FBSyxDQUFDQyxJQUFNQSxFQUFFQyxVQUFVLENBQUM7SUFFNUIsSUFBSSxDQUFDTixZQUFZO1FBQ2YsSUFBSU4sNERBQVVBLElBQUk7WUFDaEIsT0FBT0oscURBQVlBLENBQUNpQixJQUFJLENBQUM7Z0JBQUVDLFVBQVVoQiw2REFBV0EsR0FBR2dCLFFBQVE7WUFBQztRQUM5RDtRQUNBLE9BQU9sQixxREFBWUEsQ0FBQ2lCLElBQUksQ0FBQztZQUFFRSxPQUFPO1FBQW9CLEdBQUc7WUFBRUMsUUFBUTtRQUFJO0lBQ3pFO0lBRUEsTUFBTUMsUUFBUVgsV0FBV0csS0FBSyxDQUFDLElBQUksQ0FBQyxFQUFFO0lBRXRDLElBQUlWLDZEQUFXQSxDQUFDa0IsUUFBUTtRQUN0QixPQUFPckIscURBQVlBLENBQUNpQixJQUFJLENBQUM7WUFBRUMsVUFBVWhCLDZEQUFXQSxHQUFHZ0IsUUFBUTtRQUFDO0lBQzlEO0lBRUEsSUFBSTtRQUNGLE1BQU1JLFVBQVVyQiwwREFBVSxDQUFDb0IsT0FBT2hCO1FBQ2xDLE9BQU9MLHFEQUFZQSxDQUFDaUIsSUFBSSxDQUFDO1lBQUVDLFVBQVVJLFFBQVFKLFFBQVE7UUFBQztJQUN4RCxFQUFFLE9BQU07UUFDTixPQUFPbEIscURBQVlBLENBQUNpQixJQUFJLENBQUM7WUFBRUUsT0FBTztRQUFnQixHQUFHO1lBQUVDLFFBQVE7UUFBSTtJQUNyRTtBQUNGIiwic291cmNlcyI6WyJEOlxcQXBwbGljYXRpb25zXFxFY28tRGV4XFxXZWJBcHBcXGFwcFxcYXBpXFx1c2VyXFxyb3V0ZS50cyJdLCJzb3VyY2VzQ29udGVudCI6WyJpbXBvcnQgeyBOZXh0UmVzcG9uc2UgfSBmcm9tIFwibmV4dC9zZXJ2ZXJcIjtcclxuaW1wb3J0IGp3dCBmcm9tIFwianNvbndlYnRva2VuXCI7XHJcbmltcG9ydCB7IGdldERlbW9BdXRoLCBpc0RlbW9Ub2tlbiB9IGZyb20gXCJAL2hlbHBlci9kZW1vQXV0aFwiO1xyXG5pbXBvcnQgeyBpc0RlbW9Nb2RlIH0gZnJvbSBcIkAvaGVscGVyL2RlbW9Nb2RlXCI7XHJcblxyXG5jb25zdCBKV1RfU0VDUkVUID0gcHJvY2Vzcy5lbnYuSldUX1NFQ1JFVCBhcyBzdHJpbmc7XHJcblxyXG5leHBvcnQgYXN5bmMgZnVuY3Rpb24gR0VUKHJlcXVlc3Q6IFJlcXVlc3QpIHtcclxuICBjb25zdCBhdXRoQ29va2llID0gcmVxdWVzdC5oZWFkZXJzXHJcbiAgICAuZ2V0KFwiY29va2llXCIpXHJcbiAgICA/LnNwbGl0KFwiOyBcIilcclxuICAgIC5maW5kKChjKSA9PiBjLnN0YXJ0c1dpdGgoXCJhdXRoX3Rva2VuPVwiKSk7XHJcblxyXG4gIGlmICghYXV0aENvb2tpZSkge1xyXG4gICAgaWYgKGlzRGVtb01vZGUoKSkge1xyXG4gICAgICByZXR1cm4gTmV4dFJlc3BvbnNlLmpzb24oeyB1c2VybmFtZTogZ2V0RGVtb0F1dGgoKS51c2VybmFtZSB9KTtcclxuICAgIH1cclxuICAgIHJldHVybiBOZXh0UmVzcG9uc2UuanNvbih7IGVycm9yOiBcIk5vdCBhdXRoZW50aWNhdGVkXCIgfSwgeyBzdGF0dXM6IDQwMSB9KTtcclxuICB9XHJcblxyXG4gIGNvbnN0IHRva2VuID0gYXV0aENvb2tpZS5zcGxpdChcIj1cIilbMV07XHJcblxyXG4gIGlmIChpc0RlbW9Ub2tlbih0b2tlbikpIHtcclxuICAgIHJldHVybiBOZXh0UmVzcG9uc2UuanNvbih7IHVzZXJuYW1lOiBnZXREZW1vQXV0aCgpLnVzZXJuYW1lIH0pO1xyXG4gIH1cclxuXHJcbiAgdHJ5IHtcclxuICAgIGNvbnN0IGRlY29kZWQgPSBqd3QudmVyaWZ5KHRva2VuLCBKV1RfU0VDUkVUKSBhcyB7IHVzZXJuYW1lOiBzdHJpbmcgfTtcclxuICAgIHJldHVybiBOZXh0UmVzcG9uc2UuanNvbih7IHVzZXJuYW1lOiBkZWNvZGVkLnVzZXJuYW1lIH0pO1xyXG4gIH0gY2F0Y2gge1xyXG4gICAgcmV0dXJuIE5leHRSZXNwb25zZS5qc29uKHsgZXJyb3I6IFwiSW52YWxpZCB0b2tlblwiIH0sIHsgc3RhdHVzOiA0MDMgfSk7XHJcbiAgfVxyXG59XHJcbiJdLCJuYW1lcyI6WyJOZXh0UmVzcG9uc2UiLCJqd3QiLCJnZXREZW1vQXV0aCIsImlzRGVtb1Rva2VuIiwiaXNEZW1vTW9kZSIsIkpXVF9TRUNSRVQiLCJwcm9jZXNzIiwiZW52IiwiR0VUIiwicmVxdWVzdCIsImF1dGhDb29raWUiLCJoZWFkZXJzIiwiZ2V0Iiwic3BsaXQiLCJmaW5kIiwiYyIsInN0YXJ0c1dpdGgiLCJqc29uIiwidXNlcm5hbWUiLCJlcnJvciIsInN0YXR1cyIsInRva2VuIiwiZGVjb2RlZCIsInZlcmlmeSJdLCJpZ25vcmVMaXN0IjpbXSwic291cmNlUm9vdCI6IiJ9\n//# sourceURL=webpack-internal:///(rsc)/./app/api/user/route.ts\n");

/***/ }),

/***/ "(rsc)/./helper/demoAuth.ts":
/*!****************************!*\
  !*** ./helper/demoAuth.ts ***!
  \****************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
eval("__webpack_require__.r(__webpack_exports__);\n/* harmony export */ __webpack_require__.d(__webpack_exports__, {\n/* harmony export */   getDemoAuth: () => (/* binding */ getDemoAuth),\n/* harmony export */   isDemoToken: () => (/* binding */ isDemoToken),\n/* harmony export */   withDemoCookies: () => (/* binding */ withDemoCookies)\n/* harmony export */ });\n/* harmony import */ var cookie__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! cookie */ \"(rsc)/./node_modules/cookie/index.js\");\n/* harmony import */ var _demoMode__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ./demoMode */ \"(rsc)/./helper/demoMode.ts\");\n\n\nfunction getDemoAuth() {\n    return {\n        id: _demoMode__WEBPACK_IMPORTED_MODULE_1__.DEMO_USER_ID,\n        username: _demoMode__WEBPACK_IMPORTED_MODULE_1__.DEMO_USERNAME\n    };\n}\nfunction withDemoCookies(response) {\n    const cookieOptions = {\n        httpOnly: true,\n        maxAge: 60 * 60 * 24 * 30,\n        path: \"/\"\n    };\n    response.headers.append(\"Set-Cookie\", (0,cookie__WEBPACK_IMPORTED_MODULE_0__.serialize)(\"auth_token\", _demoMode__WEBPACK_IMPORTED_MODULE_1__.DEMO_AUTH_TOKEN, cookieOptions));\n    response.headers.append(\"Set-Cookie\", (0,cookie__WEBPACK_IMPORTED_MODULE_0__.serialize)(\"username\", _demoMode__WEBPACK_IMPORTED_MODULE_1__.DEMO_USERNAME, cookieOptions));\n    return response;\n}\nfunction isDemoToken(token) {\n    return (0,_demoMode__WEBPACK_IMPORTED_MODULE_1__.isDemoMode)() && token === _demoMode__WEBPACK_IMPORTED_MODULE_1__.DEMO_AUTH_TOKEN;\n}\n//# sourceURL=[module]\n//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiKHJzYykvLi9oZWxwZXIvZGVtb0F1dGgudHMiLCJtYXBwaW5ncyI6Ijs7Ozs7Ozs7QUFDbUM7QUFNZjtBQUViLFNBQVNLO0lBQ2QsT0FBTztRQUFFQyxJQUFJSCxtREFBWUE7UUFBRUksVUFBVUwsb0RBQWFBO0lBQUM7QUFDckQ7QUFFTyxTQUFTTSxnQkFBZ0JDLFFBQXNCO0lBQ3BELE1BQU1DLGdCQUFnQjtRQUNwQkMsVUFBVTtRQUNWQyxRQUFRLEtBQUssS0FBSyxLQUFLO1FBQ3ZCQyxNQUFNO0lBQ1I7SUFFQUosU0FBU0ssT0FBTyxDQUFDQyxNQUFNLENBQ3JCLGNBQ0FmLGlEQUFTQSxDQUFDLGNBQWNDLHNEQUFlQSxFQUFFUztJQUUzQ0QsU0FBU0ssT0FBTyxDQUFDQyxNQUFNLENBQ3JCLGNBQ0FmLGlEQUFTQSxDQUFDLFlBQVlFLG9EQUFhQSxFQUFFUTtJQUd2QyxPQUFPRDtBQUNUO0FBRU8sU0FBU08sWUFBWUMsS0FBYztJQUN4QyxPQUFPYixxREFBVUEsTUFBTWEsVUFBVWhCLHNEQUFlQTtBQUNsRCIsInNvdXJjZXMiOlsiRDpcXEFwcGxpY2F0aW9uc1xcRWNvLURleFxcV2ViQXBwXFxoZWxwZXJcXGRlbW9BdXRoLnRzIl0sInNvdXJjZXNDb250ZW50IjpbImltcG9ydCB7IE5leHRSZXNwb25zZSB9IGZyb20gXCJuZXh0L3NlcnZlclwiO1xyXG5pbXBvcnQgeyBzZXJpYWxpemUgfSBmcm9tIFwiY29va2llXCI7XHJcbmltcG9ydCB7XHJcbiAgREVNT19BVVRIX1RPS0VOLFxyXG4gIERFTU9fVVNFUk5BTUUsXHJcbiAgREVNT19VU0VSX0lELFxyXG4gIGlzRGVtb01vZGUsXHJcbn0gZnJvbSBcIi4vZGVtb01vZGVcIjtcclxuXHJcbmV4cG9ydCBmdW5jdGlvbiBnZXREZW1vQXV0aCgpIHtcclxuICByZXR1cm4geyBpZDogREVNT19VU0VSX0lELCB1c2VybmFtZTogREVNT19VU0VSTkFNRSB9O1xyXG59XHJcblxyXG5leHBvcnQgZnVuY3Rpb24gd2l0aERlbW9Db29raWVzKHJlc3BvbnNlOiBOZXh0UmVzcG9uc2UpIHtcclxuICBjb25zdCBjb29raWVPcHRpb25zID0ge1xyXG4gICAgaHR0cE9ubHk6IHRydWUsXHJcbiAgICBtYXhBZ2U6IDYwICogNjAgKiAyNCAqIDMwLFxyXG4gICAgcGF0aDogXCIvXCIsXHJcbiAgfTtcclxuXHJcbiAgcmVzcG9uc2UuaGVhZGVycy5hcHBlbmQoXHJcbiAgICBcIlNldC1Db29raWVcIixcclxuICAgIHNlcmlhbGl6ZShcImF1dGhfdG9rZW5cIiwgREVNT19BVVRIX1RPS0VOLCBjb29raWVPcHRpb25zKVxyXG4gICk7XHJcbiAgcmVzcG9uc2UuaGVhZGVycy5hcHBlbmQoXHJcbiAgICBcIlNldC1Db29raWVcIixcclxuICAgIHNlcmlhbGl6ZShcInVzZXJuYW1lXCIsIERFTU9fVVNFUk5BTUUsIGNvb2tpZU9wdGlvbnMpXHJcbiAgKTtcclxuXHJcbiAgcmV0dXJuIHJlc3BvbnNlO1xyXG59XHJcblxyXG5leHBvcnQgZnVuY3Rpb24gaXNEZW1vVG9rZW4odG9rZW4/OiBzdHJpbmcpIHtcclxuICByZXR1cm4gaXNEZW1vTW9kZSgpICYmIHRva2VuID09PSBERU1PX0FVVEhfVE9LRU47XHJcbn1cclxuIl0sIm5hbWVzIjpbInNlcmlhbGl6ZSIsIkRFTU9fQVVUSF9UT0tFTiIsIkRFTU9fVVNFUk5BTUUiLCJERU1PX1VTRVJfSUQiLCJpc0RlbW9Nb2RlIiwiZ2V0RGVtb0F1dGgiLCJpZCIsInVzZXJuYW1lIiwid2l0aERlbW9Db29raWVzIiwicmVzcG9uc2UiLCJjb29raWVPcHRpb25zIiwiaHR0cE9ubHkiLCJtYXhBZ2UiLCJwYXRoIiwiaGVhZGVycyIsImFwcGVuZCIsImlzRGVtb1Rva2VuIiwidG9rZW4iXSwiaWdub3JlTGlzdCI6W10sInNvdXJjZVJvb3QiOiIifQ==\n//# sourceURL=webpack-internal:///(rsc)/./helper/demoAuth.ts\n");

/***/ }),

/***/ "(rsc)/./helper/demoMode.ts":
/*!****************************!*\
  !*** ./helper/demoMode.ts ***!
  \****************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
eval("__webpack_require__.r(__webpack_exports__);\n/* harmony export */ __webpack_require__.d(__webpack_exports__, {\n/* harmony export */   DEMO_AUTH_TOKEN: () => (/* binding */ DEMO_AUTH_TOKEN),\n/* harmony export */   DEMO_USERNAME: () => (/* binding */ DEMO_USERNAME),\n/* harmony export */   DEMO_USER_ID: () => (/* binding */ DEMO_USER_ID),\n/* harmony export */   isDemoMode: () => (/* binding */ isDemoMode)\n/* harmony export */ });\nfunction isPlaceholderMongoUri(uri) {\n    if (!uri) return true;\n    return uri.includes(\"YOUR_USER\") || uri.includes(\"YOUR_PASSWORD\") || uri.includes(\"cluster0.ev0ma.mongodb.net\");\n}\nconst isDemoMode = ()=>{\n    if (true) return true;\n    if (false) {}\n    return isPlaceholderMongoUri(process.env.MONGODB_URI || \"\");\n};\nconst DEMO_AUTH_TOKEN = \"demo-mode\";\nconst DEMO_USERNAME = \"RAG-ED Demo\";\nconst DEMO_USER_ID = \"demo-user-id\";\n//# sourceURL=[module]\n//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiKHJzYykvLi9oZWxwZXIvZGVtb01vZGUudHMiLCJtYXBwaW5ncyI6Ijs7Ozs7OztBQUFBLFNBQVNBLHNCQUFzQkMsR0FBVztJQUN4QyxJQUFJLENBQUNBLEtBQUssT0FBTztJQUNqQixPQUNFQSxJQUFJQyxRQUFRLENBQUMsZ0JBQ2JELElBQUlDLFFBQVEsQ0FBQyxvQkFDYkQsSUFBSUMsUUFBUSxDQUFDO0FBRWpCO0FBRU8sTUFBTUMsYUFBYTtJQUN4QixJQUFJQyxJQUFnQyxFQUFFLE9BQU87SUFDN0MsSUFBSUEsS0FBaUMsRUFBRSxFQUFhO0lBQ3BELE9BQU9KLHNCQUFzQkksUUFBUUMsR0FBRyxDQUFDRSxXQUFXLElBQUk7QUFDMUQsRUFBRTtBQUVLLE1BQU1DLGtCQUFrQixZQUFZO0FBQ3BDLE1BQU1DLGdCQUFnQixjQUFjO0FBQ3BDLE1BQU1DLGVBQWUsZUFBZSIsInNvdXJjZXMiOlsiRDpcXEFwcGxpY2F0aW9uc1xcRWNvLURleFxcV2ViQXBwXFxoZWxwZXJcXGRlbW9Nb2RlLnRzIl0sInNvdXJjZXNDb250ZW50IjpbImZ1bmN0aW9uIGlzUGxhY2Vob2xkZXJNb25nb1VyaSh1cmk6IHN0cmluZykge1xyXG4gIGlmICghdXJpKSByZXR1cm4gdHJ1ZTtcclxuICByZXR1cm4gKFxyXG4gICAgdXJpLmluY2x1ZGVzKFwiWU9VUl9VU0VSXCIpIHx8XHJcbiAgICB1cmkuaW5jbHVkZXMoXCJZT1VSX1BBU1NXT1JEXCIpIHx8XHJcbiAgICB1cmkuaW5jbHVkZXMoXCJjbHVzdGVyMC5ldjBtYS5tb25nb2RiLm5ldFwiKVxyXG4gICk7XHJcbn1cclxuXHJcbmV4cG9ydCBjb25zdCBpc0RlbW9Nb2RlID0gKCkgPT4ge1xyXG4gIGlmIChwcm9jZXNzLmVudi5ERU1PX01PREUgPT09IFwidHJ1ZVwiKSByZXR1cm4gdHJ1ZTtcclxuICBpZiAocHJvY2Vzcy5lbnYuREVNT19NT0RFID09PSBcImZhbHNlXCIpIHJldHVybiBmYWxzZTtcclxuICByZXR1cm4gaXNQbGFjZWhvbGRlck1vbmdvVXJpKHByb2Nlc3MuZW52Lk1PTkdPREJfVVJJIHx8IFwiXCIpO1xyXG59O1xyXG5cclxuZXhwb3J0IGNvbnN0IERFTU9fQVVUSF9UT0tFTiA9IFwiZGVtby1tb2RlXCI7XHJcbmV4cG9ydCBjb25zdCBERU1PX1VTRVJOQU1FID0gXCJSQUctRUQgRGVtb1wiO1xyXG5leHBvcnQgY29uc3QgREVNT19VU0VSX0lEID0gXCJkZW1vLXVzZXItaWRcIjtcclxuIl0sIm5hbWVzIjpbImlzUGxhY2Vob2xkZXJNb25nb1VyaSIsInVyaSIsImluY2x1ZGVzIiwiaXNEZW1vTW9kZSIsInByb2Nlc3MiLCJlbnYiLCJERU1PX01PREUiLCJNT05HT0RCX1VSSSIsIkRFTU9fQVVUSF9UT0tFTiIsIkRFTU9fVVNFUk5BTUUiLCJERU1PX1VTRVJfSUQiXSwiaWdub3JlTGlzdCI6W10sInNvdXJjZVJvb3QiOiIifQ==\n//# sourceURL=webpack-internal:///(rsc)/./helper/demoMode.ts\n");

/***/ })

};
;

// load runtime
var __webpack_require__ = require("../../../webpack-runtime.js");
__webpack_require__.C(exports);
var __webpack_exec__ = (moduleId) => (__webpack_require__(__webpack_require__.s = moduleId))
var __webpack_exports__ = __webpack_require__.X(0, ["vendor-chunks/next","vendor-chunks/semver","vendor-chunks/jsonwebtoken","vendor-chunks/lodash.includes","vendor-chunks/cookie","vendor-chunks/jws","vendor-chunks/lodash.once","vendor-chunks/jwa","vendor-chunks/lodash.isinteger","vendor-chunks/ecdsa-sig-formatter","vendor-chunks/lodash.isplainobject","vendor-chunks/ms","vendor-chunks/lodash.isstring","vendor-chunks/lodash.isnumber","vendor-chunks/lodash.isboolean","vendor-chunks/safe-buffer","vendor-chunks/buffer-equal-constant-time"], () => (__webpack_exec__("(rsc)/./node_modules/next/dist/build/webpack/loaders/next-app-loader/index.js?name=app%2Fapi%2Fuser%2Froute&page=%2Fapi%2Fuser%2Froute&appPaths=&pagePath=private-next-app-dir%2Fapi%2Fuser%2Froute.ts&appDir=D%3A%5CApplications%5CEco-Dex%5CWebApp%5Capp&pageExtensions=tsx&pageExtensions=ts&pageExtensions=jsx&pageExtensions=js&rootDir=D%3A%5CApplications%5CEco-Dex%5CWebApp&isDev=true&tsconfigPath=tsconfig.json&basePath=&assetPrefix=&nextConfigOutput=&preferredRegion=&middlewareConfig=e30%3D!")));
module.exports = __webpack_exports__;

})();