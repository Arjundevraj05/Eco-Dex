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
exports.id = "app/api/save-username/route";
exports.ids = ["app/api/save-username/route"];
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

/***/ "(rsc)/./node_modules/next/dist/build/webpack/loaders/next-app-loader/index.js?name=app%2Fapi%2Fsave-username%2Froute&page=%2Fapi%2Fsave-username%2Froute&appPaths=&pagePath=private-next-app-dir%2Fapi%2Fsave-username%2Froute.ts&appDir=D%3A%5CApplications%5CEco-Dex%5CWebApp%5Capp&pageExtensions=tsx&pageExtensions=ts&pageExtensions=jsx&pageExtensions=js&rootDir=D%3A%5CApplications%5CEco-Dex%5CWebApp&isDev=true&tsconfigPath=tsconfig.json&basePath=&assetPrefix=&nextConfigOutput=&preferredRegion=&middlewareConfig=e30%3D!":
/*!*****************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************!*\
  !*** ./node_modules/next/dist/build/webpack/loaders/next-app-loader/index.js?name=app%2Fapi%2Fsave-username%2Froute&page=%2Fapi%2Fsave-username%2Froute&appPaths=&pagePath=private-next-app-dir%2Fapi%2Fsave-username%2Froute.ts&appDir=D%3A%5CApplications%5CEco-Dex%5CWebApp%5Capp&pageExtensions=tsx&pageExtensions=ts&pageExtensions=jsx&pageExtensions=js&rootDir=D%3A%5CApplications%5CEco-Dex%5CWebApp&isDev=true&tsconfigPath=tsconfig.json&basePath=&assetPrefix=&nextConfigOutput=&preferredRegion=&middlewareConfig=e30%3D! ***!
  \*****************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
eval("__webpack_require__.r(__webpack_exports__);\n/* harmony export */ __webpack_require__.d(__webpack_exports__, {\n/* harmony export */   patchFetch: () => (/* binding */ patchFetch),\n/* harmony export */   routeModule: () => (/* binding */ routeModule),\n/* harmony export */   serverHooks: () => (/* binding */ serverHooks),\n/* harmony export */   workAsyncStorage: () => (/* binding */ workAsyncStorage),\n/* harmony export */   workUnitAsyncStorage: () => (/* binding */ workUnitAsyncStorage)\n/* harmony export */ });\n/* harmony import */ var next_dist_server_route_modules_app_route_module_compiled__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! next/dist/server/route-modules/app-route/module.compiled */ \"(rsc)/./node_modules/next/dist/server/route-modules/app-route/module.compiled.js\");\n/* harmony import */ var next_dist_server_route_modules_app_route_module_compiled__WEBPACK_IMPORTED_MODULE_0___default = /*#__PURE__*/__webpack_require__.n(next_dist_server_route_modules_app_route_module_compiled__WEBPACK_IMPORTED_MODULE_0__);\n/* harmony import */ var next_dist_server_route_kind__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! next/dist/server/route-kind */ \"(rsc)/./node_modules/next/dist/server/route-kind.js\");\n/* harmony import */ var next_dist_server_lib_patch_fetch__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! next/dist/server/lib/patch-fetch */ \"(rsc)/./node_modules/next/dist/server/lib/patch-fetch.js\");\n/* harmony import */ var next_dist_server_lib_patch_fetch__WEBPACK_IMPORTED_MODULE_2___default = /*#__PURE__*/__webpack_require__.n(next_dist_server_lib_patch_fetch__WEBPACK_IMPORTED_MODULE_2__);\n/* harmony import */ var D_Applications_Eco_Dex_WebApp_app_api_save_username_route_ts__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! ./app/api/save-username/route.ts */ \"(rsc)/./app/api/save-username/route.ts\");\n\n\n\n\n// We inject the nextConfigOutput here so that we can use them in the route\n// module.\nconst nextConfigOutput = \"\"\nconst routeModule = new next_dist_server_route_modules_app_route_module_compiled__WEBPACK_IMPORTED_MODULE_0__.AppRouteRouteModule({\n    definition: {\n        kind: next_dist_server_route_kind__WEBPACK_IMPORTED_MODULE_1__.RouteKind.APP_ROUTE,\n        page: \"/api/save-username/route\",\n        pathname: \"/api/save-username\",\n        filename: \"route\",\n        bundlePath: \"app/api/save-username/route\"\n    },\n    resolvedPagePath: \"D:\\\\Applications\\\\Eco-Dex\\\\WebApp\\\\app\\\\api\\\\save-username\\\\route.ts\",\n    nextConfigOutput,\n    userland: D_Applications_Eco_Dex_WebApp_app_api_save_username_route_ts__WEBPACK_IMPORTED_MODULE_3__\n});\n// Pull out the exports that we need to expose from the module. This should\n// be eliminated when we've moved the other routes to the new format. These\n// are used to hook into the route.\nconst { workAsyncStorage, workUnitAsyncStorage, serverHooks } = routeModule;\nfunction patchFetch() {\n    return (0,next_dist_server_lib_patch_fetch__WEBPACK_IMPORTED_MODULE_2__.patchFetch)({\n        workAsyncStorage,\n        workUnitAsyncStorage\n    });\n}\n\n\n//# sourceMappingURL=app-route.js.map//# sourceURL=[module]\n//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiKHJzYykvLi9ub2RlX21vZHVsZXMvbmV4dC9kaXN0L2J1aWxkL3dlYnBhY2svbG9hZGVycy9uZXh0LWFwcC1sb2FkZXIvaW5kZXguanM/bmFtZT1hcHAlMkZhcGklMkZzYXZlLXVzZXJuYW1lJTJGcm91dGUmcGFnZT0lMkZhcGklMkZzYXZlLXVzZXJuYW1lJTJGcm91dGUmYXBwUGF0aHM9JnBhZ2VQYXRoPXByaXZhdGUtbmV4dC1hcHAtZGlyJTJGYXBpJTJGc2F2ZS11c2VybmFtZSUyRnJvdXRlLnRzJmFwcERpcj1EJTNBJTVDQXBwbGljYXRpb25zJTVDRWNvLURleCU1Q1dlYkFwcCU1Q2FwcCZwYWdlRXh0ZW5zaW9ucz10c3gmcGFnZUV4dGVuc2lvbnM9dHMmcGFnZUV4dGVuc2lvbnM9anN4JnBhZ2VFeHRlbnNpb25zPWpzJnJvb3REaXI9RCUzQSU1Q0FwcGxpY2F0aW9ucyU1Q0Vjby1EZXglNUNXZWJBcHAmaXNEZXY9dHJ1ZSZ0c2NvbmZpZ1BhdGg9dHNjb25maWcuanNvbiZiYXNlUGF0aD0mYXNzZXRQcmVmaXg9Jm5leHRDb25maWdPdXRwdXQ9JnByZWZlcnJlZFJlZ2lvbj0mbWlkZGxld2FyZUNvbmZpZz1lMzAlM0QhIiwibWFwcGluZ3MiOiI7Ozs7Ozs7Ozs7Ozs7O0FBQStGO0FBQ3ZDO0FBQ3FCO0FBQ29CO0FBQ2pHO0FBQ0E7QUFDQTtBQUNBLHdCQUF3Qix5R0FBbUI7QUFDM0M7QUFDQSxjQUFjLGtFQUFTO0FBQ3ZCO0FBQ0E7QUFDQTtBQUNBO0FBQ0EsS0FBSztBQUNMO0FBQ0E7QUFDQSxZQUFZO0FBQ1osQ0FBQztBQUNEO0FBQ0E7QUFDQTtBQUNBLFFBQVEsc0RBQXNEO0FBQzlEO0FBQ0EsV0FBVyw0RUFBVztBQUN0QjtBQUNBO0FBQ0EsS0FBSztBQUNMO0FBQzBGOztBQUUxRiIsInNvdXJjZXMiOlsiIl0sInNvdXJjZXNDb250ZW50IjpbImltcG9ydCB7IEFwcFJvdXRlUm91dGVNb2R1bGUgfSBmcm9tIFwibmV4dC9kaXN0L3NlcnZlci9yb3V0ZS1tb2R1bGVzL2FwcC1yb3V0ZS9tb2R1bGUuY29tcGlsZWRcIjtcbmltcG9ydCB7IFJvdXRlS2luZCB9IGZyb20gXCJuZXh0L2Rpc3Qvc2VydmVyL3JvdXRlLWtpbmRcIjtcbmltcG9ydCB7IHBhdGNoRmV0Y2ggYXMgX3BhdGNoRmV0Y2ggfSBmcm9tIFwibmV4dC9kaXN0L3NlcnZlci9saWIvcGF0Y2gtZmV0Y2hcIjtcbmltcG9ydCAqIGFzIHVzZXJsYW5kIGZyb20gXCJEOlxcXFxBcHBsaWNhdGlvbnNcXFxcRWNvLURleFxcXFxXZWJBcHBcXFxcYXBwXFxcXGFwaVxcXFxzYXZlLXVzZXJuYW1lXFxcXHJvdXRlLnRzXCI7XG4vLyBXZSBpbmplY3QgdGhlIG5leHRDb25maWdPdXRwdXQgaGVyZSBzbyB0aGF0IHdlIGNhbiB1c2UgdGhlbSBpbiB0aGUgcm91dGVcbi8vIG1vZHVsZS5cbmNvbnN0IG5leHRDb25maWdPdXRwdXQgPSBcIlwiXG5jb25zdCByb3V0ZU1vZHVsZSA9IG5ldyBBcHBSb3V0ZVJvdXRlTW9kdWxlKHtcbiAgICBkZWZpbml0aW9uOiB7XG4gICAgICAgIGtpbmQ6IFJvdXRlS2luZC5BUFBfUk9VVEUsXG4gICAgICAgIHBhZ2U6IFwiL2FwaS9zYXZlLXVzZXJuYW1lL3JvdXRlXCIsXG4gICAgICAgIHBhdGhuYW1lOiBcIi9hcGkvc2F2ZS11c2VybmFtZVwiLFxuICAgICAgICBmaWxlbmFtZTogXCJyb3V0ZVwiLFxuICAgICAgICBidW5kbGVQYXRoOiBcImFwcC9hcGkvc2F2ZS11c2VybmFtZS9yb3V0ZVwiXG4gICAgfSxcbiAgICByZXNvbHZlZFBhZ2VQYXRoOiBcIkQ6XFxcXEFwcGxpY2F0aW9uc1xcXFxFY28tRGV4XFxcXFdlYkFwcFxcXFxhcHBcXFxcYXBpXFxcXHNhdmUtdXNlcm5hbWVcXFxccm91dGUudHNcIixcbiAgICBuZXh0Q29uZmlnT3V0cHV0LFxuICAgIHVzZXJsYW5kXG59KTtcbi8vIFB1bGwgb3V0IHRoZSBleHBvcnRzIHRoYXQgd2UgbmVlZCB0byBleHBvc2UgZnJvbSB0aGUgbW9kdWxlLiBUaGlzIHNob3VsZFxuLy8gYmUgZWxpbWluYXRlZCB3aGVuIHdlJ3ZlIG1vdmVkIHRoZSBvdGhlciByb3V0ZXMgdG8gdGhlIG5ldyBmb3JtYXQuIFRoZXNlXG4vLyBhcmUgdXNlZCB0byBob29rIGludG8gdGhlIHJvdXRlLlxuY29uc3QgeyB3b3JrQXN5bmNTdG9yYWdlLCB3b3JrVW5pdEFzeW5jU3RvcmFnZSwgc2VydmVySG9va3MgfSA9IHJvdXRlTW9kdWxlO1xuZnVuY3Rpb24gcGF0Y2hGZXRjaCgpIHtcbiAgICByZXR1cm4gX3BhdGNoRmV0Y2goe1xuICAgICAgICB3b3JrQXN5bmNTdG9yYWdlLFxuICAgICAgICB3b3JrVW5pdEFzeW5jU3RvcmFnZVxuICAgIH0pO1xufVxuZXhwb3J0IHsgcm91dGVNb2R1bGUsIHdvcmtBc3luY1N0b3JhZ2UsIHdvcmtVbml0QXN5bmNTdG9yYWdlLCBzZXJ2ZXJIb29rcywgcGF0Y2hGZXRjaCwgIH07XG5cbi8vIyBzb3VyY2VNYXBwaW5nVVJMPWFwcC1yb3V0ZS5qcy5tYXAiXSwibmFtZXMiOltdLCJpZ25vcmVMaXN0IjpbXSwic291cmNlUm9vdCI6IiJ9\n//# sourceURL=webpack-internal:///(rsc)/./node_modules/next/dist/build/webpack/loaders/next-app-loader/index.js?name=app%2Fapi%2Fsave-username%2Froute&page=%2Fapi%2Fsave-username%2Froute&appPaths=&pagePath=private-next-app-dir%2Fapi%2Fsave-username%2Froute.ts&appDir=D%3A%5CApplications%5CEco-Dex%5CWebApp%5Capp&pageExtensions=tsx&pageExtensions=ts&pageExtensions=jsx&pageExtensions=js&rootDir=D%3A%5CApplications%5CEco-Dex%5CWebApp&isDev=true&tsconfigPath=tsconfig.json&basePath=&assetPrefix=&nextConfigOutput=&preferredRegion=&middlewareConfig=e30%3D!\n");

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

/***/ "(rsc)/./app/api/save-username/route.ts":
/*!****************************************!*\
  !*** ./app/api/save-username/route.ts ***!
  \****************************************/
/***/ ((__unused_webpack_module, __webpack_exports__, __webpack_require__) => {

"use strict";
eval("__webpack_require__.r(__webpack_exports__);\n/* harmony export */ __webpack_require__.d(__webpack_exports__, {\n/* harmony export */   POST: () => (/* binding */ POST)\n/* harmony export */ });\n/* harmony import */ var next_server__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! next/server */ \"(rsc)/./node_modules/next/dist/api/server.js\");\n/* harmony import */ var _helper_demoMode__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! @/helper/demoMode */ \"(rsc)/./helper/demoMode.ts\");\n\n\nasync function POST(request) {\n    try {\n        if ((0,_helper_demoMode__WEBPACK_IMPORTED_MODULE_1__.isDemoMode)()) {\n            return next_server__WEBPACK_IMPORTED_MODULE_0__.NextResponse.json({\n                success: true\n            });\n        }\n        const { username } = await request.json();\n        if (!username) {\n            return next_server__WEBPACK_IMPORTED_MODULE_0__.NextResponse.json({\n                error: \"Username is required\"\n            }, {\n                status: 400\n            });\n        }\n        const response = next_server__WEBPACK_IMPORTED_MODULE_0__.NextResponse.json({\n            success: true\n        });\n        response.cookies.set(\"username\", username, {\n            httpOnly: true,\n            maxAge: 24 * 60 * 60\n        });\n        return response;\n    } catch (error) {\n        console.error(\"Error saving username:\", error);\n        return next_server__WEBPACK_IMPORTED_MODULE_0__.NextResponse.json({\n            error: \"Failed to save username\"\n        }, {\n            status: 500\n        });\n    }\n}\n//# sourceURL=[module]\n//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiKHJzYykvLi9hcHAvYXBpL3NhdmUtdXNlcm5hbWUvcm91dGUudHMiLCJtYXBwaW5ncyI6Ijs7Ozs7O0FBQXdEO0FBQ1Q7QUFFeEMsZUFBZUUsS0FBS0MsT0FBb0I7SUFDN0MsSUFBSTtRQUNGLElBQUlGLDREQUFVQSxJQUFJO1lBQ2hCLE9BQU9ELHFEQUFZQSxDQUFDSSxJQUFJLENBQUM7Z0JBQUVDLFNBQVM7WUFBSztRQUMzQztRQUVBLE1BQU0sRUFBRUMsUUFBUSxFQUFFLEdBQUcsTUFBTUgsUUFBUUMsSUFBSTtRQUV2QyxJQUFJLENBQUNFLFVBQVU7WUFDYixPQUFPTixxREFBWUEsQ0FBQ0ksSUFBSSxDQUFDO2dCQUFFRyxPQUFPO1lBQXVCLEdBQUc7Z0JBQUVDLFFBQVE7WUFBSTtRQUM1RTtRQUVBLE1BQU1DLFdBQVdULHFEQUFZQSxDQUFDSSxJQUFJLENBQUM7WUFBRUMsU0FBUztRQUFLO1FBQ25ESSxTQUFTQyxPQUFPLENBQUNDLEdBQUcsQ0FBQyxZQUFZTCxVQUFVO1lBQUVNLFVBQVU7WUFBTUMsUUFBUSxLQUFLLEtBQUs7UUFBRztRQUVsRixPQUFPSjtJQUNULEVBQUUsT0FBT0YsT0FBTztRQUNkTyxRQUFRUCxLQUFLLENBQUMsMEJBQTBCQTtRQUN4QyxPQUFPUCxxREFBWUEsQ0FBQ0ksSUFBSSxDQUFDO1lBQUVHLE9BQU87UUFBMEIsR0FBRztZQUFFQyxRQUFRO1FBQUk7SUFDL0U7QUFDRiIsInNvdXJjZXMiOlsiRDpcXEFwcGxpY2F0aW9uc1xcRWNvLURleFxcV2ViQXBwXFxhcHBcXGFwaVxcc2F2ZS11c2VybmFtZVxccm91dGUudHMiXSwic291cmNlc0NvbnRlbnQiOlsiaW1wb3J0IHsgTmV4dFJlcXVlc3QsIE5leHRSZXNwb25zZSB9IGZyb20gXCJuZXh0L3NlcnZlclwiO1xyXG5pbXBvcnQgeyBpc0RlbW9Nb2RlIH0gZnJvbSBcIkAvaGVscGVyL2RlbW9Nb2RlXCI7XHJcblxyXG5leHBvcnQgYXN5bmMgZnVuY3Rpb24gUE9TVChyZXF1ZXN0OiBOZXh0UmVxdWVzdCkge1xyXG4gIHRyeSB7XHJcbiAgICBpZiAoaXNEZW1vTW9kZSgpKSB7XHJcbiAgICAgIHJldHVybiBOZXh0UmVzcG9uc2UuanNvbih7IHN1Y2Nlc3M6IHRydWUgfSk7XHJcbiAgICB9XHJcblxyXG4gICAgY29uc3QgeyB1c2VybmFtZSB9ID0gYXdhaXQgcmVxdWVzdC5qc29uKCk7XHJcblxyXG4gICAgaWYgKCF1c2VybmFtZSkge1xyXG4gICAgICByZXR1cm4gTmV4dFJlc3BvbnNlLmpzb24oeyBlcnJvcjogXCJVc2VybmFtZSBpcyByZXF1aXJlZFwiIH0sIHsgc3RhdHVzOiA0MDAgfSk7XHJcbiAgICB9XHJcblxyXG4gICAgY29uc3QgcmVzcG9uc2UgPSBOZXh0UmVzcG9uc2UuanNvbih7IHN1Y2Nlc3M6IHRydWUgfSk7XHJcbiAgICByZXNwb25zZS5jb29raWVzLnNldChcInVzZXJuYW1lXCIsIHVzZXJuYW1lLCB7IGh0dHBPbmx5OiB0cnVlLCBtYXhBZ2U6IDI0ICogNjAgKiA2MCB9KTtcclxuXHJcbiAgICByZXR1cm4gcmVzcG9uc2U7XHJcbiAgfSBjYXRjaCAoZXJyb3IpIHtcclxuICAgIGNvbnNvbGUuZXJyb3IoXCJFcnJvciBzYXZpbmcgdXNlcm5hbWU6XCIsIGVycm9yKTtcclxuICAgIHJldHVybiBOZXh0UmVzcG9uc2UuanNvbih7IGVycm9yOiBcIkZhaWxlZCB0byBzYXZlIHVzZXJuYW1lXCIgfSwgeyBzdGF0dXM6IDUwMCB9KTtcclxuICB9XHJcbn1cclxuIl0sIm5hbWVzIjpbIk5leHRSZXNwb25zZSIsImlzRGVtb01vZGUiLCJQT1NUIiwicmVxdWVzdCIsImpzb24iLCJzdWNjZXNzIiwidXNlcm5hbWUiLCJlcnJvciIsInN0YXR1cyIsInJlc3BvbnNlIiwiY29va2llcyIsInNldCIsImh0dHBPbmx5IiwibWF4QWdlIiwiY29uc29sZSJdLCJpZ25vcmVMaXN0IjpbXSwic291cmNlUm9vdCI6IiJ9\n//# sourceURL=webpack-internal:///(rsc)/./app/api/save-username/route.ts\n");

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
var __webpack_exports__ = __webpack_require__.X(0, ["vendor-chunks/next"], () => (__webpack_exec__("(rsc)/./node_modules/next/dist/build/webpack/loaders/next-app-loader/index.js?name=app%2Fapi%2Fsave-username%2Froute&page=%2Fapi%2Fsave-username%2Froute&appPaths=&pagePath=private-next-app-dir%2Fapi%2Fsave-username%2Froute.ts&appDir=D%3A%5CApplications%5CEco-Dex%5CWebApp%5Capp&pageExtensions=tsx&pageExtensions=ts&pageExtensions=jsx&pageExtensions=js&rootDir=D%3A%5CApplications%5CEco-Dex%5CWebApp&isDev=true&tsconfigPath=tsconfig.json&basePath=&assetPrefix=&nextConfigOutput=&preferredRegion=&middlewareConfig=e30%3D!")));
module.exports = __webpack_exports__;

})();